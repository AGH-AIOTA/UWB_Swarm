#include "dw3000.h"

/*
 * TDoA — BLINK NODE (TAG)
 *
 * W TDoA tag jest stroną pasywną obliczeniowo — nadaje jeden blink,
 * resztę robią listenery.
 *
 * Schemat:
 *
 *   Tag:     |──Blink TX──|                     (1 ramka, żadnej odpowiedzi)
 *   Listen1:              |──odbiór o T1──|──UART report──|
 *   Listen2:              |──odbiór o T2──|──UART report──|
 *   Listen3:              |──odbiór o T3──|──UART report──|
 *
 *   PC/RPi zbiera T1, T2, T3 → TDoA₁₂ = T1-T2, TDoA₁₃ = T1-T3
 *   → przecięcie hiperboli → pozycja tagu
 *
 * Wymagania systemu TDoA:
 *   - min. 3 listenery (2D), 4 listenery (3D)
 *   - listenery zsynchronizowane (patrz listener_node.cpp i docs/swarm_sync.md)
 *   - znane pozycje listenerów (z pomiaru lub GPS)
 *
 * Ramka Blink (10 bajtów + 2 CRC):
 *   bajt 0-1: 0x4188 — frame control (data, 16-bit addressing)
 *   bajt 2:   numer sekwencji (inkrementowany)
 *   bajt 3-4: 0xDECA — PAN ID
 *   bajt 5-6: unikalny adres tagu (TAG_ID — zmień per pojazd!)
 *   bajt 7-8: adres broadcast 0xFFFF
 *   bajt 9:   0xE3 — kod funkcji "Blink"
 *   bajt 10-11: CRC (auto)
 *
 * WAŻNE: każdy pojazd/tag musi mieć unikalny TAG_ID — listenery
 * rozróżniają blinki od różnych tagów właśnie po tym polu.
 */

#define APP_NAME "TDoA BLINK NODE v1.0"

// === KONFIGURACJA POJAZDU — zmień na unikalną wartość dla każdego tagu ===
#define TAG_ID 0x0001  // unikalny adres 16-bit: 0x0001, 0x0002, 0x0003, ...

// Częstotliwość blinkowania — kompromis: szybko = więcej kolizji przy N tagach
#define BLINK_PERIOD_MS 100   // 10 Hz — dla małego roju (< 5 tagów)
// Przy N tagach i TDMA: każdy tag dostaje slot BLINK_PERIOD_MS/N
// Patrz docs/swarm_sync.md sekcja TDMA

const uint8_t PIN_RST = D3;
const uint8_t PIN_IRQ = D0;
const uint8_t PIN_SS  = D7;

// Konfiguracja UWB — identyczna we wszystkich węzłach sieci
static dwt_config_t config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1,
    DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
    (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

#define TX_ANT_DLY 16395
#define RX_ANT_DLY 16395

/*
 * Ramka Blink z zakodowanym TAG_ID w polach adresowych.
 *   bajt 5-6: adres źródłowy = TAG_ID (little-endian)
 *   bajt 7-8: adres docelowy = 0xFFFF (broadcast)
 *
 * Listenery wyciągają TAG_ID z bajtów 5-6 żeby wiedzieć który tag blinkuje.
 */
static uint8_t tx_blink_msg[] = {
    0x41, 0x88,                         // frame control
    0,                                   // bajt 2: sequence number (wypełniany dynamicznie)
    0xCA, 0xDE,                          // bajt 3-4: PAN ID
    (uint8_t)(TAG_ID & 0xFF),            // bajt 5: TAG_ID lo byte
    (uint8_t)((TAG_ID >> 8) & 0xFF),     // bajt 6: TAG_ID hi byte
    0xFF, 0xFF,                          // bajt 7-8: broadcast
    0xE3,                                // bajt 9: kod funkcji "Blink"
    0, 0                                 // bajt 10-11: CRC (auto)
};

#define BLINK_MSG_SN_IDX 2

static uint8_t blink_seq = 0;

extern dwt_txconfig_t txconfig_options;

void setup() {
    UART_init();
    test_run_info((unsigned char *)APP_NAME);

    Serial.print("Tag ID: 0x");
    Serial.println(TAG_ID, HEX);

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

    // Blink node nie ustawia RX — tylko nadaje, nie nasłuchuje odpowiedzi
}

void loop() {
    // Wstaw numer sekwencji — listenery używają go do dopasowania odczytów
    // od różnych listenerów do tego samego blinku (ten sam seq = ten sam blink)
    tx_blink_msg[BLINK_MSG_SN_IDX] = blink_seq++;

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK); // wyczyść flagę TX
    dwt_writetxdata(sizeof(tx_blink_msg), tx_blink_msg, 0);
    dwt_writetxfctrl(sizeof(tx_blink_msg), 0, 1); // ranging bit = 1 (precyzyjny RMARKER)

    // Zwykły TX bez odpowiedzi — w TDoA blink jest jednostronny
    dwt_starttx(DWT_START_TX_IMMEDIATE);

    // Czekaj aż frame zostanie wysłany przed przejściem do Sleep
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {};
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

    Sleep(BLINK_PERIOD_MS);
}
