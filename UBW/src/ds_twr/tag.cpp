#include "dw3000.h"

/*
 * DS-TWR INITIATOR (TAG) — Double-Sided Two-Way Ranging
 *
 * DS-TWR używa 3 ramek zamiast 2 jak SS-TWR:
 *
 *   Tag:    |--Poll TX (T1)--|         |--Resp RX (T4)--|--Final TX (T5)--|
 *   Anchor: |--Poll RX (T2)--|--Resp TX (T3)--|         |--Final RX (T6)--|
 *
 * Każda ze stron mierzy swój "round-trip" i czas odpowiedzi:
 *   Ra = T4 - T1  (round-trip tagu,   zegar tagu)
 *   Da = T5 - T4  (czas odpow. tagu,  zegar tagu)
 *   Db = T3 - T2  (czas odpow. anch., zegar anchora)
 *   Rb = T6 - T3  (round-trip anch.,  zegar anchora)
 *
 * Formuła DS-TWR — eliminuje dryft zegarów do rzędu ε²:
 *   TOF = (Ra × Rb - Da × Db) / (Ra + Rb + Da + Db)
 *
 * Dlaczego to działa?
 *   W SS-TWR błąd ≈ ε × T_reply (rząd ε, np. 2.4 m przy 20 PPM).
 *   W DS-TWR błąd ≈ ε² × T_reply → dla ε=20 PPM daje ~48 µm = pomijalny.
 *   Błąd się zeruje bo mierzymy dwa round-tripy różnymi zegarami
 *   i efekt rozciągnięcia/skrócenia czasu działa w przeciwnych kierunkach.
 *
 * Podział pracy:
 *   Tag: wysyła Poll, odbiera Response, wysyła Final (z Ra i Da w środku)
 *   Anchor: odbiera wszystko, sam oblicza TOF i drukuje dystans
 *   → tag nie dostaje wyniku w tej 3-ramkowej wersji
 */

#define APP_NAME "DS TWR TAG v1.0"

const uint8_t PIN_RST = D3;
const uint8_t PIN_IRQ = D0;
const uint8_t PIN_SS  = D7;

// Konfiguracja UWB — identyczna jak w anchor.cpp (inaczej nie będą się słyszeć)
static dwt_config_t config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1,
    DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
    (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

// Kalibracja — identyczna jak w anchor.cpp (oba urządzenia muszą mieć tę samą wartość)
#define ANT_DLY_BASE  16395
#define ANT_DLY_CALIB    17   // +17 DTU ≈ +16 cm korekcja; patrz anchor.cpp

#define TX_ANT_DLY (ANT_DLY_BASE + ANT_DLY_CALIB)
#define RX_ANT_DLY (ANT_DLY_BASE + ANT_DLY_CALIB)

// Czas od TX Poll do otwarcia okna RX na Response
#define POLL_TX_TO_RESP_RX_DLY_UUS   500
// Timeout na odebranie Response
#define RESP_RX_TIMEOUT_UUS          800
// Czas od RX Response do TX Final — musi być >= czas potrzebny MCU na obliczenia
// Im krótszy tym lepszy dla dokładności, ale ESP32-C3 potrzebuje trochę czasu
#define RESP_RX_TO_FINAL_TX_DLY_UUS  1000

#define RNG_DELAY_MS 1000

// Struktura ramek IEEE 802.15.4 (pierwsze 10 bajtów wspólne)
#define ALL_MSG_COMMON_LEN  10
#define ALL_MSG_SN_IDX       2

// Indeksy pól w ramce Final: Ra i Da wpisywane przez tag, odczytywane przez anchor
#define FINAL_MSG_RA_IDX    10  // bajty 10–13: Ra = T4-T1 [DTU, uint32 LE]
#define FINAL_MSG_DA_IDX    14  // bajty 14–17: Da = T5-T4 [DTU, uint32 LE]

#define RX_BUF_LEN 12  // Response to prosta ramka: 10 bajtów nagłówka + 2 CRC

/*
 * Trzy rodzaje ramek — każda ma inny bajt funkcji (byte[9]):
 *   0xE0 = Poll    (tag → anchor)
 *   0xE1 = Response (anchor → tag)
 *   0xE2 = Final   (tag → anchor, z Ra i Da w bajtach 10–17)
 *
 * bajt:  0     1    2     3     4    5    6    7    8    9   [10..17]  18  19
 */
static uint8_t tx_poll_msg[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8_t rx_resp_msg[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0};
static uint8_t tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE2,
                                  0, 0, 0, 0,   // bajty 10–13: Ra  (wypełniane dynamicznie)
                                  0, 0, 0, 0,   // bajty 14–17: Da  (wypełniane dynamicznie)
                                  0, 0};        // bajty 18–19: CRC (auto)

static uint8_t  frame_seq_nb = 0;
static uint8_t  rx_buffer[RX_BUF_LEN];
static uint32_t status_reg = 0;

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

    // Te ustawienia dotyczą okna RX po wysłaniu Poll (czekamy na Response).
    // Po wysłaniu Final NIE czekamy na nic — Response_expected nie jest ustawiane dla Final TX.
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
}

void loop() {
    // =========================================================================
    // RAMKA 1 — Poll TX
    // Cel: wyzwolić wymianę, ustawić T1 (sprzętowo przez DW3000)
    // =========================================================================

    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK); // wyczyść flagę TX done
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); // ostatni arg=1: ramka rangingowa (ustawia RMARKER)

    // DWT_RESPONSE_EXPECTED: chip automatycznie włączy RX po TX
    // po czasie POLL_TX_TO_RESP_RX_DLY_UUS, z timeoutem RESP_RX_TIMEOUT_UUS
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    // Busy-wait: czekaj na dobrą ramkę (RXFCG) LUB timeout LUB błąd
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {};

    frame_seq_nb++;

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
        // Anchor nie odpowiedział lub ramka uszkodzona — pomiń tę iterację
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        Sleep(RNG_DELAY_MS);
        return;
    }

    // =========================================================================
    // RAMKA 2 — Response RX
    // Cel: odebrać potwierdzenie od anchora, zarejestrować T4
    // =========================================================================

    uint32_t frame_len;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

    if (frame_len > sizeof(rx_buffer)) { Sleep(RNG_DELAY_MS); return; }

    dwt_readrxdata(rx_buffer, frame_len, 0);
    rx_buffer[ALL_MSG_SN_IDX] = 0; // wyzeruj SN przed porównaniem
    if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) != 0) { Sleep(RNG_DELAY_MS); return; }

    // T1: timestamp TX Poll (lo32 — bezpieczne bo T4-T1 < 67 ms = 2^32 DTU)
    uint32_t poll_tx_ts = dwt_readtxtimestamplo32();

    // T4: timestamp RX Response — potrzebujemy pełnych 64 bitów do obliczenia T5
    uint64_t resp_rx_ts_64 = get_rx_timestamp_u64();
    uint32_t resp_rx_ts    = (uint32_t)resp_rx_ts_64; // lo32 dla Ra

    // Ra = T4 - T1: round-trip po stronie tagu (oba timestampy z zegara tagu)
    uint32_t Ra = resp_rx_ts - poll_tx_ts;

    // =========================================================================
    // RAMKA 3 — Final TX
    // Cel: poinformować anchor o Ra i Da żeby mógł policzyć TOF
    // =========================================================================

    // Oblicz T5 z wyprzedzeniem — identyczna technika jak anchor oblicza T3 w SS-TWR.
    // Musimy znać T5 PRZED wysłaniem, bo wpisujemy Da=T5-T4 do treści ramki.
    uint32_t final_tx_time = (resp_rx_ts_64 + ((uint64_t)RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    // >> 8: DW3000 przechowuje delayed-TX time w formacie 32-bit = górne 32 bity 40-bitowego timestampa
    dwt_setdelayedtrxtime(final_tx_time); // zaprogramuj hardware żeby nadał dokładnie o tym czasie

    // Odtwórz pełny 40-bitowy T5 z 32-bitowego final_tx_time + offset anteny TX
    // (& 0xFFFFFFFE: zeruj bit 0 — wymagane wyrównanie do 512 DTU)
    uint64_t final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

    // Da = T5 - T4: czas odpowiedzi tagu (oba timestampy z zegara tagu)
    // Obliczamy z 32-bitowego lo32 obu timestampów — wrapping uint32 zadziała poprawnie
    uint32_t Da = (uint32_t)final_tx_ts - resp_rx_ts;

    // Wpisz Ra i Da do ramki Final (anchor odczyta je po odebraniu)
    memcpy(&tx_final_msg[FINAL_MSG_RA_IDX], &Ra, 4); // Ra: little-endian uint32
    memcpy(&tx_final_msg[FINAL_MSG_DA_IDX], &Da, 4); // Da: little-endian uint32

    tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
    dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1); // ranging bit = 1

    // DWT_START_TX_DELAYED: nadaj dokładnie o zaplanowanym czasie final_tx_time
    // Bez DWT_RESPONSE_EXPECTED — w tej wersji nie czekamy na wynik od anchora
    int ret = dwt_starttx(DWT_START_TX_DELAYED);

    if (ret == DWT_SUCCESS) {
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {};
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        frame_seq_nb++;
        Serial.println("DS-TWR exchange complete — see anchor serial for distance");
    } else {
        // Opóźnienie było za krótkie — MCU wywołał starttx po zaprogramowanym czasie
        // Anchor wykryje brak Final przez timeout i wróci do nasłuchu
        Serial.println("Final TX late — increase RESP_RX_TO_FINAL_TX_DLY_UUS");
    }

    Sleep(RNG_DELAY_MS);
}
