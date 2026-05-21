#include "dw3000.h"

/*
 * DS-TWR RESPONDER (ANCHOR) — Double-Sided Two-Way Ranging
 *
 * Schemat wymiany i timestampy:
 *
 *   Tag:    |--Poll TX (T1)--|          |--Resp RX (T4)--|--Final TX (T5)--|
 *   Anchor:         |--Poll RX (T2)--|--Resp TX (T3)--|          |--Final RX (T6)--|
 *
 * Timestampy anchora (mierzone lokalnym zegarem):
 *   T2 = czas odbioru Poll
 *   T3 = czas nadania Response  (obliczany z wyprzedzeniem jak w SS-TWR)
 *   T6 = czas odbioru Final
 *
 * Timestampy tagu (przesyłane w ramce Final):
 *   Ra = T4 - T1  (round-trip tagu)
 *   Da = T5 - T4  (czas odpowiedzi tagu)
 *
 * Co anchor oblicza lokalnie:
 *   Db = T3 - T2  (czas odpowiedzi anchora, zegar anchora)
 *   Rb = T6 - T3  (round-trip anchora,      zegar anchora)
 *
 * Formuła TOF — eliminuje dryft zegarów do rzędu ε²:
 *   TOF = (Ra × Rb  −  Da × Db) / (Ra + Rb + Da + Db)
 *
 * Dlaczego ten wzór niweluje dryft?
 *   Ra i Da mierzone zegarem A (tagu).
 *   Rb i Db mierzone zegarem B (anchora, biegnącym szybciej o ε).
 *   Iloczyn Ra×Rb zawiera czynnik (1+ε), podobnie Da×Db zawiera (1+ε).
 *   W różnicy (Ra×Rb − Da×Db) czynnik (1+ε) się wyciąga przed nawias
 *   i skraca z (1+ε) w mianowniku — zostaje błąd rzędu ε² ≈ (20 PPM)² → ~48 µm.
 *   Porównaj z SS-TWR gdzie błąd to ε × T_reply ≈ 2.4 m.
 */

#define APP_NAME "DS TWR ANCHOR v1.0"

const uint8_t PIN_RST = D3;
const uint8_t PIN_IRQ = D0;
const uint8_t PIN_SS  = D7;

// Konfiguracja UWB — identyczna jak w tag.cpp
static dwt_config_t config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1,
    DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
    (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

/*
 * KALIBRACJA OPÓŹNIENIA ANTENY
 *
 * Skąd bierze się stały offset dystansu:
 *   DW3000 dodaje TX_ANT_DLY do timestampa TX (sygnał wychodzi później niż chip "myśli")
 *   DW3000 odejmuje RX_ANT_DLY od timestampa RX (sygnał dotarł do anteny wcześniej niż chip zarejestrował)
 *   Razem modelują fizyczne opóźnienie PCB + kabla + anteny.
 *   Jeśli te wartości nie pasują do rzeczywistości → stały błąd odległości.
 *
 * Jak obliczyć ANT_DLY_CALIB:
 *   1. Ustaw oba urządzenia w dokładnie zmierzonej odległości d_ref (np. 1.000 m,
 *      mierz od centrum anteny do centrum anteny)
 *   2. Odczytaj zmierzoną odległość d_meas
 *   3. error_m = d_ref - d_meas   (dodatni = za krótki, ujemny = za długi)
 *   4. ANT_DLY_CALIB = error_m / (SPEED_OF_LIGHT * DWT_TIME_UNITS) / 2
 *
 * Dla bieżącego przypadku: d_meas jest o 0.16 m za krótki:
 *   ANT_DLY_CALIB = 0.16 / (299702547.0 * 15.65e-12) / 2 ≈ +17 DTU
 *
 * UWAGA: wartość musi być ustawiona IDENTYCZNIE na tagu i anchorze.
 * Zmień ANT_DLY_CALIB, skompiluj i wgraj na oba urządzenia.
 * Jeśli po zmianie błąd wzrósł (zamiast maleć) — zmień znak na ujemny.
 */
#define ANT_DLY_BASE  16395   // typowa wartość fabryczna dla DW3000 + zewnętrzna antena
#define ANT_DLY_CALIB    17   // korekcja: +17 DTU ≈ +16 cm (zmierz i dostosuj!)

#define TX_ANT_DLY (ANT_DLY_BASE + ANT_DLY_CALIB)
#define RX_ANT_DLY (ANT_DLY_BASE + ANT_DLY_CALIB)

// Czas od RX Poll do TX Response (jak w SS-TWR; im krótszy tym lepiej)
#define POLL_RX_TO_RESP_TX_DLY_UUS   800

// Czas od TX Response do otwarcia okna RX na Final.
// Final przylatuje ~RESP_RX_TO_FINAL_TX_DLY_UUS (1000 µs) po Response TX.
// Ustawiamy 700 µs żeby okno otworzyć przed przybyciem Final.
#define RESP_TX_TO_FINAL_RX_DLY_UUS  700

// Timeout na odebranie Final od momentu otwarcia okna RX.
// Musi obejmować czas transmisji Final (~300 µs przy 6.8 Mbps) + jitter.
#define FINAL_RX_TIMEOUT_UUS         500

// Indeksy pól w ramkach
#define ALL_MSG_COMMON_LEN  10
#define ALL_MSG_SN_IDX       2
#define FINAL_MSG_RA_IDX    10  // bajty 10–13 ramki Final: Ra od tagu
#define FINAL_MSG_DA_IDX    14  // bajty 14–17 ramki Final: Da od tagu

// Anchor odbiera Poll (12 B) i Final (20 B) — bufor musi mieścić największą
#define RX_BUF_LEN 20

/*
 * Ramki:
 *   bajt:  0     1    2     3     4    5    6    7    8    9   [10..17]  18  19
 *   Poll:  0x41 0x88  SN  0xCA 0xDE  'W'  'A'  'V'  'E' 0xE0   --      CRC
 *   Resp:  0x41 0x88  SN  0xCA 0xDE  'V'  'E'  'W'  'A' 0xE1   --      CRC
 *   Final: 0x41 0x88  SN  0xCA 0xDE  'W'  'A'  'V'  'E' 0xE2  Ra  Da   CRC
 */
static uint8_t rx_poll_msg[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8_t tx_resp_msg[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0};
static uint8_t rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint8_t  frame_seq_nb = 0;
static uint8_t  rx_buffer[RX_BUF_LEN];
static uint32_t status_reg = 0;

static uint64_t poll_rx_ts; // T2: pełny 40-bitowy timestamp odbioru Poll
static uint64_t resp_tx_ts; // T3: pełny 40-bitowy timestamp nadania Response (obliczany z wyprzedzeniem)

extern dwt_txconfig_t txconfig_options;

void setup() {
    UART_init();
    test_run_info((unsigned char *)APP_NAME);

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

    // setrxaftertxdelay i setrxtimeout ustawiamy dynamicznie przed Response TX
    // (zob. pętla loop), bo dla Poll czekamy bez timeoutu a dla Final z timeoutem

    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
}

void loop() {
    // =========================================================================
    // RAMKA 1 — Poll RX
    // Cel: odebrać wyzwolenie od tagu, zarejestrować T2
    // =========================================================================

    // Wyłącz timeout — na Poll czekamy w nieskończoność (anchor jest pasywny)
    dwt_setrxtimeout(0);
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
    rx_buffer[ALL_MSG_SN_IDX] = 0;
    if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) != 0) return;

    // T2: pełny 40-bitowy timestamp odbioru Poll — potrzebny do obliczenia T3 i Db
    poll_rx_ts = get_rx_timestamp_u64();

    // =========================================================================
    // RAMKA 2 — Response TX
    // Cel: potwierdzić Poll, wyzwolić wysłanie Final przez tag
    // T3 obliczamy z wyprzedzeniem (identyczna technika jak T3 w SS-TWR anchor)
    // =========================================================================

    // Oblicz czas TX: T3 = T2 + opóźnienie; >> 8 bo rejestr delayed-TX to górne 32 bity
    uint32_t resp_tx_time = (poll_rx_ts + ((uint64_t)POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);

    // Odtwórz pełny 40-bitowy T3 (potrzebny do Db = T3-T2 i Rb = T6-T3)
    // & 0xFFFFFFFE: wymagane wyrównanie do 512 DTU (bit 0 musi być 0)
    // + TX_ANT_DLY: kompensacja opóźnienia anteny TX
    resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

    // Ustaw okno RX po TX Response dla odbioru Final:
    // setrxaftertxdelay: kiedy chip włączy RX po zakończeniu TX Response
    // setrxtimeout: jak długo czekać na Final
    // Muszą być ustawione PRZED starttx, bo dotyczą automatycznego RX po TX
    dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
    dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

    tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
    dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); // ranging bit = 1

    // DWT_START_TX_DELAYED: nadaj dokładnie w czasie resp_tx_time
    // DWT_RESPONSE_EXPECTED: po TX automatycznie włącz RX (dla Final)
    int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

    if (ret != DWT_SUCCESS) {
        // MCU spóźnił się z wywołaniem starttx — porzuć tę wymianę
        return;
    }

    // Czekaj na potwierdzenie wysłania Response
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {};
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    frame_seq_nb++;

    // =========================================================================
    // RAMKA 3 — Final RX
    // Cel: odebrać Ra i Da od tagu, zarejestrować T6, obliczyć TOF
    // =========================================================================

    // Chip automatycznie włączył RX po Response TX (DWT_RESPONSE_EXPECTED).
    // Czekamy na Final z timeoutem FINAL_RX_TIMEOUT_UUS.
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {};

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
        // Final nie przyszedł lub błąd CRC — tag prawdopodobnie nie zdążył
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return;
    }

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

    if (frame_len > sizeof(rx_buffer)) return;

    dwt_readrxdata(rx_buffer, frame_len, 0);
    rx_buffer[ALL_MSG_SN_IDX] = 0;
    if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) != 0) return;

    // T6: timestamp odbioru Final — potrzebny do Rb = T6 - T3
    uint64_t final_rx_ts = get_rx_timestamp_u64();

    // Odczytaj Ra i Da z payload ramki Final (wpisane przez tag jako uint32 LE)
    uint32_t Ra, Da;
    memcpy(&Ra, &rx_buffer[FINAL_MSG_RA_IDX], 4); // Ra = T4 - T1 (zegar tagu)
    memcpy(&Da, &rx_buffer[FINAL_MSG_DA_IDX], 4); // Da = T5 - T4 (zegar tagu)

    // Oblicz Db i Rb z lokalnych timestampów anchora
    // Używamy lo32 — różnice są < 67 ms więc 32-bitowe odejmowanie nie przepełni
    uint32_t Db = (uint32_t)resp_tx_ts - (uint32_t)poll_rx_ts; // T3 - T2 (zegar anchora)
    uint32_t Rb = (uint32_t)final_rx_ts - (uint32_t)resp_tx_ts; // T6 - T3 (zegar anchora)

    // =========================================================================
    // Obliczenie TOF — formuła DS-TWR
    // =========================================================================
    //
    //         Ra × Rb  −  Da × Db
    // TOF =  ─────────────────────────
    //         Ra + Rb  +  Da + Db
    //
    // Używamy double żeby uniknąć przepełnienia przy mnożeniu uint32 × uint32
    // (max wartość ~2^32 × 2^32 = 2^64 — nie mieści się w uint64)
    double tof      = ((double)Ra * Rb - (double)Da * Db)
                    / ((double)Ra + Rb + Da + Db)
                    * DWT_TIME_UNITS; // przelicznik DTU → sekundy (1 DTU ≈ 15.65 ps)
    double distance = tof * SPEED_OF_LIGHT;

    Serial.print("DS-TWR Distance: ");
    Serial.print(distance, 4);
    Serial.println(" m");
}
