#include "dw3000.h"

/*
 * TDoA — LISTENER NODE (ANCHOR)
 *
 * Listener odbiera blinki od tagów, zapisuje precyzyjny timestamp odbioru
 * i wysyła raport przez UART do hosta (PC/RPi).
 *
 * Format raportu UART (jedna linia na blink):
 *   "TDOA,<tag_id_hex>,<seq>,<listener_id>,<timestamp_lo32>\n"
 *   np.: "TDOA,0001,42,2,1234567890\n"
 *
 * Host zbiera raporty ze wszystkich listenerów i dla każdego (tag_id, seq)
 * oblicza TDoA = timestamp_L1 - timestamp_L2, itd. → pozycja przez NLLS.
 *
 * === WYMAGANIE: synchronizacja zegarów listenerów ===
 *
 * Surowe timestampy każdego listenera są w jego lokalnym zegarze — żeby
 * różnica T_L1 - T_L2 reprezentowała fizyczną różnicę czasu przybycia,
 * oba zegary muszą być skalibrowane względem wspólnej osi czasu.
 *
 * Metoda synchronizacji implementowana tutaj:
 *   SYNC_MASTER (listener 0) nadaje co SYNC_PERIOD_MS ramkę SyncBroadcast.
 *   Każdy inny listener odbiera sync, oblicza offset:
 *     offset = T_sync_oczekiwany - T_sync_zmierzony
 *   i dodaje go do każdego mierzonego timestampu.
 *
 *   Oczekiwany timestamp sync = poprzedni sync + SYNC_PERIOD_MS w DTU.
 *   Przy pierwszym odczycie inicjalizujemy na 0 i czekamy na drugi sync.
 *
 * Dokładność tej metody: ~10–50 ns (zależy od regularności SYNC_PERIOD).
 * Dla lepszej dokładności patrz docs/swarm_sync.md — TWR-based sync.
 *
 * WAŻNE: każdy listener musi mieć unikalny LISTENER_ID.
 */

#define APP_NAME "TDoA LISTENER v1.0"

// === KONFIGURACJA — zmień dla każdego listenera ===
#define LISTENER_ID   1       // unikalny: 0 = sync master, 1, 2, 3, ...
#define SYNC_MASTER   0       // ID listenera pełniącego rolę sync mastera

// Listener 0 nadaje sync co tyle ms; reszta nasłuchuje
#define SYNC_PERIOD_MS 1000
// Kod funkcji w ramce sync (musi się różnić od blinku)
#define FUNC_BLINK  0xE3
#define FUNC_SYNC   0xE4

const uint8_t PIN_RST = D3;
const uint8_t PIN_IRQ = D0;
const uint8_t PIN_SS  = D7;

static dwt_config_t config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1,
    DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
    (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

#define TX_ANT_DLY 16395
#define RX_ANT_DLY 16395

#define ALL_MSG_SN_IDX   2
#define BLINK_SRC_IDX    5   // bajty 5-6: adres źródłowy = TAG_ID
#define FUNC_CODE_IDX    9   // bajt 9: kod funkcji

#define RX_BUF_LEN 20

// Ramka sync mastera — broadcast co SYNC_PERIOD_MS
// Sync master (listener 0) nadaje tę ramkę; reszta ją odbiera i kalibruje zegar.
static uint8_t tx_sync_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE,
    (uint8_t)(SYNC_MASTER & 0xFF), (uint8_t)((SYNC_MASTER >> 8) & 0xFF), // src = master ID
    0xFF, 0xFF,                    // dst = broadcast
    FUNC_SYNC,                     // kod funkcji sync
    0, 0                           // CRC
};

static uint8_t  rx_buffer[RX_BUF_LEN];
static uint32_t status_reg = 0;
static uint8_t  sync_seq   = 0;

// Korekcja dryfu zegara tego listenera względem mastera
// Dodawana do każdego lokalnego timestampu przed wysłaniem raportu
static int64_t  clock_offset_dtu = 0;  // [DTU]

// Poprzedni timestamp sync — potrzebny do obliczenia dryfu między syncami
static uint64_t last_sync_ts  = 0;
static bool     sync_initialized = false;

extern dwt_txconfig_t txconfig_options;

// -----------------------------------------------------------------------
// Wyślij sync broadcast (tylko listener 0)
// -----------------------------------------------------------------------
static void send_sync() {
    tx_sync_msg[ALL_MSG_SN_IDX] = sync_seq++;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_sync_msg), tx_sync_msg, 0);
    dwt_writetxfctrl(sizeof(tx_sync_msg), 0, 1);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {};
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
}

// -----------------------------------------------------------------------
// Zaktualizuj offset zegara na podstawie odbioru ramki sync
// Metoda: porównaj oczekiwany a rzeczywisty timestamp odbioru sync.
// -----------------------------------------------------------------------
static void update_clock_offset(uint64_t received_sync_ts) {
    if (!sync_initialized) {
        // Pierwsza ramka sync — tylko zapamiętaj timestamp, czekaj na drugą
        last_sync_ts     = received_sync_ts;
        sync_initialized = true;
        return;
    }

    // Oczekiwany odstęp między syncami w DTU
    // (SYNC_PERIOD_MS * 1000 µs * UUS_TO_DWT_TIME / 65536)
    // Uproszczenie: 1 ms ≈ 64000000/1000 DTU — liczymy przez UUS_TO_DWT_TIME
    int64_t expected_delta = (int64_t)SYNC_PERIOD_MS * 1000LL * UUS_TO_DWT_TIME / 65536LL;

    int64_t actual_delta   = (int64_t)(received_sync_ts - last_sync_ts);

    // Offset = ile DTU nasz zegar "pędzi" lub "spóźnia" względem mastera
    // Akumulujemy — po każdym syncu dodajemy deltę błędu
    clock_offset_dtu += (expected_delta - actual_delta);

    last_sync_ts = received_sync_ts;
}

void setup() {
    UART_init();
    test_run_info((unsigned char *)APP_NAME);

    Serial.print("Listener ID: ");
    Serial.println(LISTENER_ID);
    Serial.println(LISTENER_ID == SYNC_MASTER ? "Role: SYNC MASTER" : "Role: slave");

    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(PIN_SS);
    delay(2);

    while (!dwt_checkidlerc()) { UART_puts("IDLE FAILED\r\n"); while (1); }
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) { UART_puts("INIT FAILED\r\n"); while (1); }

    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
    if (dwt_configure(&config)) { UART_puts("CONFIG FAILED\r\n"); while (1); }

    dwt_configuretxrf(&txconfig_options);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    // Listener RX bez timeoutu — czeka na dowolną ramkę
    dwt_setrxtimeout(0);
}

void loop() {
    // Sync master nadaje sync co SYNC_PERIOD_MS, potem wraca do nasłuchu
#if LISTENER_ID == SYNC_MASTER
    static uint32_t last_sync_wall_ms = 0;
    uint32_t now = millis();
    if (now - last_sync_wall_ms >= SYNC_PERIOD_MS) {
        last_sync_wall_ms = now;
        send_sync();
        // Po TX wróć do nasłuchu (RX włączone poniżej)
    }
#endif

    // Włącz RX i czekaj na dowolną ramkę (blink lub sync)
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {};

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        return;
    }

    uint32_t frame_len;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

    if (frame_len > sizeof(rx_buffer)) return;
    dwt_readrxdata(rx_buffer, frame_len, 0);

    uint8_t func_code = rx_buffer[FUNC_CODE_IDX];
    uint8_t seq       = rx_buffer[ALL_MSG_SN_IDX];

    // Odczytaj timestamp odbioru — używamy pełnego 64-bit dla precyzji
    uint64_t rx_ts = get_rx_timestamp_u64();

    if (func_code == FUNC_SYNC) {
        // ---------------------------------------------------------------
        // Ramka sync od mastera — zaktualizuj korekcję zegara
        // ---------------------------------------------------------------
        if (LISTENER_ID != SYNC_MASTER) {
            update_clock_offset(rx_ts);
        }

    } else if (func_code == FUNC_BLINK) {
        // ---------------------------------------------------------------
        // Ramka blink od tagu — zaraportuj timestamp do hosta
        // ---------------------------------------------------------------

        // Wyciągnij TAG_ID z pól adresowych (bajty 5-6, little-endian)
        uint16_t tag_id = (uint16_t)rx_buffer[BLINK_SRC_IDX] |
                          ((uint16_t)rx_buffer[BLINK_SRC_IDX + 1] << 8);

        // Zastosuj korekcję dryfu — sprowadza timestamp do osi czasu mastera
        // Uwaga: lo32 wystarczy do obliczania TDoA bo różnice < 67 ms
        uint32_t corrected_ts = (uint32_t)rx_ts + (uint32_t)clock_offset_dtu;

        // Raport do hosta — format CSV parsowany przez skrypt PC/RPi
        // "TDOA,<tag_id>,<seq>,<listener_id>,<corrected_timestamp>"
        Serial.print("TDOA,");
        Serial.print(tag_id, HEX);
        Serial.print(",");
        Serial.print(seq);
        Serial.print(",");
        Serial.print(LISTENER_ID);
        Serial.print(",");
        Serial.println(corrected_ts);
    }
}
