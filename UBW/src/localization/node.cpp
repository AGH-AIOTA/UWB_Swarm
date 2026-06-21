#include "dw3000.h"
#include <string.h>

/*
 * UWB 2D RELATIVE LOCALIZATION — SWARM DEMO
 *
 * Ten sam firmware na wszystkich modułach — zmień tylko MY_ID (0..N-1).
 *
 * SLOTY WOLNOBIEŻNE (free-running, BEZ globalnego SYNC):
 *   slot = (millis() / SLOT_MS) % N_NODES
 *   - W SWOIM slocie: moduł pyta SS-TWR każdego innego (mierzy cały wiersz)
 *     i rozgłasza swoje dystanse (FUNC_DIST).
 *   - POZA swoim slotem: nasłuchuje i ODPOWIADA na POLL-e + zbiera broadcasty.
 *
 * Zegary nie są zsynchronizowane → sloty dryfują, ale bo poza slotem moduł
 * prawie zawsze słucha, większość par i tak się zmierzy (kolizje rzadkie).
 *
 * Moduł 0 zbiera całą macierz (własne pomiary + broadcasty) i drukuje:
 *   "MATRIX,d01,d02,d03,d12,d13,d23\n"   [metry]   — PC robi z tego MDS.
 *
 * Do pełnej rekonstrukcji 2D (MDS) trzeba WSZYSTKICH 6 par → min. 3 moduły
 * (najlepiej 4). Z 2 modułami zmierzysz tylko d01.
 */

// =========================================================================
// KONFIGURACJA — ZMIEŃ MY_ID PER MODUŁ
// =========================================================================
#define MY_ID     0       // ← ZMIEŃ: 0, 1, 2 lub 3
#define N_NODES   4

const uint8_t PIN_RST = D3;
const uint8_t PIN_IRQ = D0;
const uint8_t PIN_SS  = D7;

static dwt_config_t config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1,
    DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
    (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

// Kalibracja anteny: +17 DTU ≈ korekcja ~16 cm (dostosuj jeśli trzeba)
#define ANT_DLY_BASE  16395
#define ANT_DLY_CALIB    17
#define TX_ANT_DLY (ANT_DLY_BASE + ANT_DLY_CALIB)
#define RX_ANT_DLY (ANT_DLY_BASE + ANT_DLY_CALIB)

// =========================================================================
// SS-TWR timing
#define POLL_TX_TO_RESP_RX_DLY_UUS  500
#define RESP_RX_TIMEOUT_UUS         800
#define POLL_RX_TO_RESP_TX_DLY_UUS  800

// =========================================================================
// KODY FUNKCJI W RAMKACH
// =========================================================================
#define FUNC_SYNC  0xA0   // broadcast od M0: start cyklu
#define FUNC_POLL  0xA1   // SS-TWR Poll:     src→dst
#define FUNC_RESP  0xA2   // SS-TWR Response: dst→src (z T2, T3 w payload)
#define FUNC_DIST  0xA3   // broadcast:       src nadaje swoje dystanse

// Indeksy pól w ramce (wspólny nagłówek 10 bajtów IEEE 802.15.4)
#define SRC_IDX      5    // bajt 5: source node ID
#define DST_IDX      7    // bajt 7: destination ID (0xFF = broadcast)
#define FUNC_IDX     9    // bajt 9: kod funkcji
#define T2_IDX      10    // bajty 10-13 w RESP: T2 (lo32)
#define T3_IDX      14    // bajty 14-17 w RESP: T3 (lo32, planned)
#define DISTS_IDX   10    // bajty 10-25 w DIST: 4×float32 = dystanse do M0..M3

#define SN_IDX       2
#define RX_BUF_LEN  30

// =========================================================================
// GLOBALNE DANE
// =========================================================================
// Macierz odległości: dist_matrix[i][j] = dystans między Mi i Mj [m]
// 0.0 oznacza "jeszcze nie zmierzono"
static float dist_matrix[N_NODES][N_NODES];

static uint8_t  rx_buffer[RX_BUF_LEN];
static uint32_t status_reg = 0;
static uint8_t  seq        = 0;

extern dwt_txconfig_t txconfig_options;

// =========================================================================
// NARZĘDZIA: wysyłanie ramek
// =========================================================================

// Natychmiastowy TX z automatycznym RX po (dla roli initiatora SS-TWR)
static void tx_immediate_rx_after(uint8_t *msg, uint16_t len) {
    msg[SN_IDX] = seq++;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(len, msg, 0);
    dwt_writetxfctrl(len, 0, 1);  // ranging = 1
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
}

// Opóźniony TX (dla roli respondenta SS-TWR)
static int tx_delayed(uint8_t *msg, uint16_t len, uint32_t delay_time) {
    msg[SN_IDX] = seq++;
    dwt_setdelayedtrxtime(delay_time);
    dwt_writetxdata(len, msg, 0);
    dwt_writetxfctrl(len, 0, 1);
    return dwt_starttx(DWT_START_TX_DELAYED);
}

// Czeka na TXFRS z limitem czasu. Zwraca true gdy TX sie zakonczyl,
// false po timeoucie. KLUCZOWE: bez limitu MCU wisi w nieskonczonosc
// jesli TXFRS nigdy nie przyjdzie -> wyglada jakby UWB zatrzymalo procesor.
static bool wait_txfrs(uint32_t timeout_ms) {
    uint32_t t0 = millis();
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
        if (millis() - t0 > timeout_ms) return false;
    }
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    return true;
}

// Broadcast bez oczekiwania na odpowiedź
static bool tx_broadcast_nowait(uint8_t *msg, uint16_t len) {
    msg[SN_IDX] = seq++;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(len, msg, 0);
    dwt_writetxfctrl(len, 0, 0);  // ranging = 0 (ramka danych)
    int ret = dwt_starttx(DWT_START_TX_IMMEDIATE);
    if (ret != DWT_SUCCESS) return false;
    return wait_txfrs(10);
}

// =========================================================================
// SS-TWR — INITIATOR
// Wysyła Poll do target_id, czeka na Response, oblicza dystans.
// Zwraca dystans [m] lub -1.0 przy błędzie/timeout.
// =========================================================================
static float ss_twr_initiate(uint8_t target_id) {
    // Poll: src=MY_ID, dst=target_id, func=FUNC_POLL
    uint8_t poll[] = {0x41, 0x88, 0, 0xCA, 0xDE,
                      MY_ID, 0, target_id, 0, FUNC_POLL, 0, 0};

    tx_immediate_rx_after(poll, sizeof(poll));
    // Chip auto-włączy RX po POLL_TX_TO_RESP_RX_DLY_UUS z timeoutem RESP_RX_TIMEOUT_UUS

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {};

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return -1.0f;
    }

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    uint32_t flen = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (flen > sizeof(rx_buffer)) return -1.0f;
    dwt_readrxdata(rx_buffer, flen, 0);

    // Walidacja: to musi być Response skierowany do nas od target_id
    if (rx_buffer[FUNC_IDX] != FUNC_RESP)    return -1.0f;
    if (rx_buffer[SRC_IDX]  != target_id)    return -1.0f;
    if (rx_buffer[DST_IDX]  != MY_ID)        return -1.0f;

    // T1, T4: nasze własne timestampy (zegar MY_ID)
    uint32_t poll_tx_ts = dwt_readtxtimestamplo32();  // T1
    uint32_t resp_rx_ts = dwt_readrxtimestamplo32();  // T4

    // clockOffsetRatio: dryft zegara target_id względem MY_ID (mierzony z nośnej)
    float clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

    // T2, T3: timestampy target_id — przesłane w payload Response
    uint32_t poll_rx_ts, resp_tx_ts;
    memcpy(&poll_rx_ts, &rx_buffer[T2_IDX], 4);  // T2
    memcpy(&resp_tx_ts, &rx_buffer[T3_IDX], 4);  // T3

    // SS-TWR z korekcją dryfu: TOF = (RTD_init - RTD_resp×(1-ε)) / 2
    int32_t rtd_init = resp_rx_ts - poll_tx_ts;
    int32_t rtd_resp = resp_tx_ts - poll_rx_ts;
    double  tof      = ((rtd_init - rtd_resp * (1.0 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
    return  (float)(tof * SPEED_OF_LIGHT);
}

// =========================================================================
// SS-TWR — RESPONDER
// Wywołaj natychmiast po odebraniu FUNC_POLL adresowanego do MY_ID.
// Używa aktualnego stanu RX hardware (timestamp już ustawiony przez chip).
// =========================================================================
static void ss_twr_respond(uint8_t requester_id) {
    // T2: odczytaj pełny timestamp odbioru Poll (chip go zapamiętał)
    uint64_t poll_rx_ts_64 = get_rx_timestamp_u64();

    // Oblicz T3 z wyprzedzeniem (jak w ds_twr/anchor.cpp)
    uint32_t resp_tx_time = (poll_rx_ts_64 + ((uint64_t)POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    uint64_t resp_tx_ts_64 = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

    // Response: src=MY_ID, dst=requester_id, payload=T2+T3
    uint8_t resp[20] = {0x41, 0x88, 0, 0xCA, 0xDE,
                        MY_ID, 0, requester_id, 0, FUNC_RESP};
    uint32_t t2 = (uint32_t)poll_rx_ts_64;
    uint32_t t3 = (uint32_t)resp_tx_ts_64;
    memcpy(&resp[T2_IDX], &t2, 4);
    memcpy(&resp[T3_IDX], &t3, 4);
    resp[18] = 0; resp[19] = 0;

    if (tx_delayed(resp, sizeof(resp), resp_tx_time) == DWT_SUCCESS) {
        wait_txfrs(10);   // z limitem czasu — nigdy nie wisi w nieskonczonosc
    }
    // Jeśli tx_delayed == DWT_ERROR: MCU spóźniony, initiator wykryje timeout
}

// =========================================================================
// BROADCAST dystansów
// Wysyła dist_matrix[MY_ID][0..N-1] jako 4×float32.
// =========================================================================
static void broadcast_distances() {
    // Header (10B) + 4×float32 (16B) + CRC placeholder (2B) = 28B
    uint8_t msg[28] = {0x41, 0x88, 0, 0xCA, 0xDE, MY_ID, 0, 0xFF, 0, FUNC_DIST};
    memcpy(&msg[DISTS_IDX], dist_matrix[MY_ID], sizeof(float) * N_NODES);
    msg[26] = 0; msg[27] = 0;
    tx_broadcast_nowait(msg, sizeof(msg));
}

// =========================================================================
// PRZETWARZAJ odebrane ramki (używane w listen_window)
// =========================================================================
static void handle_rx_frame() {
    if (rx_buffer[FUNC_IDX] == FUNC_POLL && rx_buffer[DST_IDX] == MY_ID) {
        // Ktoś robi SS-TWR z nami — odpowiedz
        ss_twr_respond(rx_buffer[SRC_IDX]);

    } else if (rx_buffer[FUNC_IDX] == FUNC_DIST) {
        // Broadcast dystansów od innego modułu — zapisz do macierzy
        uint8_t src = rx_buffer[SRC_IDX];
        if (src < N_NODES) {
            float dists[N_NODES];
            memcpy(dists, &rx_buffer[DISTS_IDX], sizeof(float) * N_NODES);
            for (int j = 0; j < N_NODES; j++) {
                if (dists[j] > 0.05f && dists[j] < 100.0f) {
                    dist_matrix[src][j] = dists[j];
                    dist_matrix[j][src] = dists[j];  // symetria
                }
            }
            // DIAG: pokaz ze odebralismy broadcast (i jego zawartosc)
            Serial.printf("RX DIST od M%d: [%.2f %.2f %.2f %.2f]\n", src,
                          dists[0], dists[1], dists[2], dists[3]);
        }
    }
    // FUNC_SYNC i FUNC_RESP ignorowane w tym kontekście
}

// =========================================================================
// OKNO NASŁUCHU — słuchaj przez max_ms ms, obsługuj przychodzące ramki
// =========================================================================
static void listen_window(uint32_t max_ms) {
    uint32_t t_start = millis();

    while (true) {
        uint32_t elapsed = millis() - t_start;
        if (elapsed >= max_ms) break;

        // Ustaw timeout = pozostały czas (1 ms ≈ 975 UUS)
        uint32_t remaining_uus = (max_ms - elapsed) * 975;
        dwt_setrxtimeout(remaining_uus > 65535 ? 65535 : remaining_uus);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
                 (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {};

        if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            break;
        }

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        uint32_t flen = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        if (flen > sizeof(rx_buffer)) { delay(1); continue; }
        dwt_readrxdata(rx_buffer, flen, 0);

        handle_rx_frame();
        delay(1);  // FreeRTOS USB CDC task dostaje czas po obsłudze ramki
    }

    // Przywróć timeout dla roli initiatora SS-TWR
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
}

// =========================================================================
// OUTPUT macierzy dystansów (tylko moduł 0)
// Format: "MATRIX,d01,d02,d03,d12,d13,d23\n"
// =========================================================================
static void output_matrix() {
    Serial.print("MATRIX");
    for (int i = 0; i < N_NODES; i++) {
        for (int j = i + 1; j < N_NODES; j++) {
            Serial.print(",");
            Serial.print(dist_matrix[i][j], 4);
        }
    }
    Serial.println();
}

// =========================================================================
// SETUP
// =========================================================================
void setup() {
    UART_init();
    delay(100);  // poczekaj na stabilizację USB CDC przed pierwszym printem
    Serial.printf("=== Node %d booting ===\n", MY_ID);
    Serial.flush();

    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(PIN_SS);
    delay(10);  // DW3000 potrzebuje >5ms po hardware reset

    Serial.println("Checking DW3000 idle...");
    Serial.flush();
    if (!dwt_checkidlerc()) {
        Serial.println("FAIL: DW3000 not idle — sprawdz SPI (D7/D8/D9/D10) i zasilanie 3.3V");
        Serial.flush();
        while (1) { delay(1000); Serial.println("IDLE FAILED"); Serial.flush(); }
    }

    Serial.println("Initialising DW3000...");
    Serial.flush();
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        Serial.println("FAIL: dwt_initialise error");
        Serial.flush();
        while (1) { delay(1000); Serial.println("INIT FAILED"); Serial.flush(); }
    }

    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
    if (dwt_configure(&config)) {
        Serial.println("FAIL: dwt_configure error");
        Serial.flush();
        while (1) { delay(1000); Serial.println("CONFIG FAILED"); Serial.flush(); }
    }

    Serial.println("DW3000 OK");

    dwt_configuretxrf(&txconfig_options);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);  // dla roli initiatora
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);               // dla roli initiatora
    // TEST: PA/LNA WYLACZONE. Generyczne plytki DW3000 nie maja zewn. PA;
    // wlaczenie DWT_PA_ENABLE ciagnie prad przy kazdym TX i sciaga zasilanie
    // -> natywny USB C3 pada. Jesli z tym serial przezyje TX -> to byl PA.
    // dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);   // <- oryginal
    // (nie wolamy setlnapamode wcale = PA/LNA pozostaja wylaczone)

    memset(dist_matrix, 0, sizeof(dist_matrix));
}

// =========================================================================
// LOOP — TDMA cykl
// Ustaw 1 zeby zobaczyc szczegolowe checkpointy (debug). 0 = tylko wyniki.
#define DEBUG_CP 0
#if DEBUG_CP
  #define CP(x) Serial.println(x)
#else
  #define CP(x)
#endif

// =========================================================================
// LOOP — sloty wolnobieżne (free-running), BEZ globalnego SYNC.
//
// Każdy moduł dzieli czas wg millis(): slot = (millis()/SLOT_MS) % N.
//   - W SWOIM slocie: inicjuje SS-TWR do wszystkich pozostałych (cały wiersz),
//     potem rozgłasza swój wiersz dystansów.
//   - POZA swoim slotem: NASŁUCHUJE i ODPOWIADA na POLL-e + zbiera broadcasty.
//
// Dlaczego bez SYNC: SS-TWR jest dwukierunkowe — wystarczy że odbiorca jest w RX
// gdy nadawca pyta. Zegary modułów nie są zsynchronizowane, więc sloty dryfują
// względem siebie, ale ponieważ poza slotem moduł PRAWIE ZAWSZE słucha,
// kolizje są rzadkie (okno nadawania ~1ms), a większość pomiarów się udaje.
// =========================================================================
#define SLOT_MS   60u

void loop() {
    const uint32_t CYCLE_MS = N_NODES * SLOT_MS;
    uint32_t phase    = millis() % CYCLE_MS;
    int      slot     = phase / SLOT_MS;
    int32_t  slot_rem = SLOT_MS - (phase % SLOT_MS);   // ile zostało do końca slotu

    if (slot == MY_ID) {
        // MÓJ SLOT — pytam każdego innego (mierzę cały swój wiersz macierzy)
        for (int j = 0; j < N_NODES; j++) {
            if (j == MY_ID) continue;
            float d = ss_twr_initiate(j);
            if (d > 0.05f && d < 100.0f) {
                dist_matrix[MY_ID][j] = d;
                dist_matrix[j][MY_ID] = d;
                Serial.printf("M%d->M%d: %.3f m\n", MY_ID, j, d);
            }
            delay(2);  // krótki guard, by odbiorca zdążył wrócić do RX
        }
        // Rozgłoś swój wiersz — żeby M0 mógł złożyć pełną macierz
        broadcast_distances();
        // Resztę slotu słuchaj (odpowiadaj jeśli ktoś zdąży zapytać)
        int32_t rem = SLOT_MS - (int32_t)(millis() % CYCLE_MS % SLOT_MS);
        if (rem > 3) listen_window((uint32_t)rem);

    } else {
        // CUDZY SLOT — nasłuchuj i odpowiadaj do końca tego slotu
        if (slot_rem > 3) listen_window((uint32_t)slot_rem);
    }

    // OUTPUT (tylko M0) — pełna macierz dla PC, nie częściej niż co 200ms
    if (MY_ID == 0) {
        static uint32_t last_print = 0;
        if (millis() - last_print > 200) {
            output_matrix();
            last_print = millis();
        }
    }
}
