#include "dw3000.h"

/*
 * SS-TWR RESPONDER (ANCHOR)
 *
 * Anchor jest stroną pasywną — czeka na Poll od tagu, odpowiada Response.
 * Schemat:
 *   Tag:    |--Poll TX (T1)--|  ............  |--Resp RX (T4)--|
 *   Anchor: |--Poll RX (T2)--|  ............  |--Resp TX (T3)--|
 *
 * Anchor musi:
 *   1. Zapisać dokładny timestamp odbioru Poll (T2)
 *   2. Z góry obliczyć kiedy nada Response (T3) — PRZED wysłaniem,
 *      bo T3 musi być wpisane w treść ramki
 *   3. Wysłać Response z T2 i T3 w środku
 *
 * Tag oblicza TOF na podstawie czterech timestampów — anchor tylko dostarcza T2 i T3.
 * Korekcja dryfu zegarów odbywa się wyłącznie po stronie tagu.
 */

#define APP_NAME "SS TWR RESP v1.0"

const uint8_t PIN_RST = D3;
const uint8_t PIN_IRQ = D0;
const uint8_t PIN_SS  = D7;

/*
 * Konfiguracja MUSI być identyczna jak w tag.cpp — inaczej ramki nie będą odbierane.
 */
static dwt_config_t config = {
    5,               // kanał 5: 6240–6739 MHz
    DWT_PLEN_128,    // preambuła 128 symboli
    DWT_PAC8,        // okno akwizycji preambuły (musi pasować do PLEN)
    9,               // kod preambuły TX
    9,               // kod preambuły RX
    1,               // niestandardowy SFD 8-symbol
    DWT_BR_6M8,      // 6.8 Mbps — minimalizuje RTD_resp = T3 - T2, co poprawia dokładność
    DWT_PHRMODE_STD,
    DWT_PHRRATE_STD,
    (129 + 8 - 8),   // SFD timeout
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0
};

// Opóźnienie anteny — identyczne jak w tagu (jeśli używasz tych samych płytek)
#define TX_ANT_DLY 16395
#define RX_ANT_DLY 16395

/*
 * Czas od odbioru Poll do nadania Response [UWB microseconds].
 * Im krótszy tym lepiej dla dokładności — mniejszy RTD_resp = mniejszy wpływ dryfu.
 * Ale musi być wystarczający żeby MCU zdążył:
 *   - odczytać timestamp T2
 *   - obliczyć T3 i czas opóźnionego TX
 *   - wpisać T2 i T3 do bufora TX
 *   - wywołać dwt_starttx()
 * Na ESP32-C3 400–500 µs było za mało → 800 µs.
 */
#define POLL_RX_TO_RESP_TX_DLY_UUS 800

// Indeksy pól w ramkach IEEE 802.15.4
#define ALL_MSG_COMMON_LEN      10
#define ALL_MSG_SN_IDX           2
#define RESP_MSG_POLL_RX_TS_IDX 10  // bajty 10–13: T2 (timestamp odbioru Poll)
#define RESP_MSG_RESP_TX_TS_IDX 14  // bajty 14–17: T3 (timestamp nadania Response)
#define RESP_MSG_TS_LEN          4
#define RX_BUF_LEN              12  // Poll ma 12 bajtów (10 wspólnych + 2 CRC)

/*
 * Ramki rangingowe Decawave/IEEE 802.15.4:
 *   bajt 0–1: 0x8841 — frame control (data frame, 16-bit addressing)
 *   bajt 2:   numer sekwencji (inkrementowany)
 *   bajt 3–4: 0xDECA — PAN ID
 *   bajt 5–6: adres docelowy ('WA' / 'VE')
 *   bajt 7–8: adres źródłowy ('VE' / 'WA')
 *   bajt 9:   kod funkcji (0xE0 = Poll, 0xE1 = Response)
 *   bajty 10–17 (tylko Response): timestampy T2 i T3
 *   ostatnie 2 bajty: CRC, wstawiane automatycznie przez DW3000
 */
//                         0     1    2     3     4    5    6    7    8    9   ...
static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//                                                                                            ^^^^^^^^^^^^^^^^^^^^
//                                                             bajty 10–17: T2 i T3 wypełniane dynamicznie

static uint8_t  frame_seq_nb = 0;
static uint8_t  rx_buffer[RX_BUF_LEN];
static uint32_t status_reg = 0;

// T2 i T3 jako 64-bitowe — DW3000 ma 40-bitowe timestampy, uint64 mieści je bez obcinania
static uint64_t poll_rx_ts; // T2: kiedy anchor odebrał Poll
static uint64_t resp_tx_ts; // T3: kiedy anchor nada Response (obliczane z wyprzedzeniem)

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
    // Anchor NIE ustawia setrxaftertxdelay ani setrxtimeout — po nadaniu wraca do nasłuchu sam
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
}

void loop() {
    // Włącz odbiornik natychmiastowo i czekaj na cokolwiek (bez timeoutu po stronie anchora)
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {};

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
        uint32_t frame_len;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        if (frame_len <= sizeof(rx_buffer)) {
            dwt_readrxdata(rx_buffer, frame_len, 0);
            rx_buffer[ALL_MSG_SN_IDX] = 0; // wyzeruj SN przed porównaniem

            if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0) {
                uint32_t resp_tx_time;
                int ret;

                // T2: odczytaj pełny 40-bitowy timestamp odbioru Poll
                // get_rx_timestamp_u64() odczytuje wszystkie 5 bajtów rejestru RX_TIME
                poll_rx_ts = get_rx_timestamp_u64();

                // Oblicz moment nadania Response: T2 + opóźnienie [DTU]
                // UUS_TO_DWT_TIME = 65536 — przelicznik UWB-µs → DTU
                // >> 8: DW3000 przechowuje delayed TX time w formacie 32-bit = górne 32 bity z 40
                //       dolne 8 bitów musi być zerem (rozdzielczość rejestru = 512 DTU = 2^9 DTU,
                //       ale chip akceptuje >> 8 z truncation do parzystości → & 0xFFFFFFFE)
                resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(resp_tx_time); // zaprogramuj czas opóźnionego TX w DW3000

                // T3: oblicz dokładny timestamp TX z wyprzedzeniem (musi trafić do ramki PRZED wysłaniem).
                // Przywracamy pełny 40-bitowy format: (resp_tx_time & 0xFFFFFFFE) << 8
                // 0xFFFFFFFE: zerujemy bit 0 (wymagane wyrównanie do 512 DTU)
                // + TX_ANT_DLY: opóźnienie anteny, tag odejmie analogiczne RX_ANT_DLY
                resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

                // Wpisz T2 i T3 do treści ramki Response (bajty 10–17)
                // Tag odczyta je po odebraniu i użyje do obliczenia TOF
                resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);  // T2
                resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);  // T3

                tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
                dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); // ranging bit = 1 (ustawia RMARKER)
                ret = dwt_starttx(DWT_START_TX_DELAYED); // nadaj dokładnie o czasie resp_tx_time

                if (ret == DWT_SUCCESS) {
                    // Czekaj na potwierdzenie wysłania (busy-wait na flagę TXFRS)
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {};
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                    frame_seq_nb++;
                }
                // Jeśli ret != DWT_SUCCESS: opóźnienie było za krótkie, MCU wywołał starttx
                // po zaprogramowanym czasie. Pomijamy tę wymianę i wracamy do nasłuchu.
                // Tag wykryje brak odpowiedzi przez timeout i wyśle kolejny Poll.
            }
        }
    } else {
        // Błąd CRC lub inne — wyczyść i wróć do nasłuchu
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    }
}
