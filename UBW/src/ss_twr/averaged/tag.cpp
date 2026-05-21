#include "dw3000.h"
#include <math.h>

/*
 * SS-TWR INITIATOR (TAG) — wariant z uśrednianiem
 *
 * Schemat wymiany:
 *   Tag:    |--Poll TX (T1)--|  ............  |--Resp RX (T4)--|
 *   Anchor: |--Poll RX (T2)--|  ............  |--Resp TX (T3)--|
 *
 * Tag mierzy round-trip ze swojej strony:   RTD_init = T4 - T1
 * Anchor mierzy swój czas odpowiedzi:       RTD_resp = T3 - T2
 * Anchor odsyła T2 i T3 wewnątrz ramki Response.
 *
 * Formuła TOF bez korekcji:
 *   TOF = (RTD_init - RTD_resp) / 2
 *
 * Formuła TOF z korekcją dryfu zegarów:
 *   TOF = (RTD_init - RTD_resp * (1 - clockOffsetRatio)) / 2
 *
 * Dlaczego korekcja: kryształy kwarcowe mają tolerancję ±20–50 PPM.
 * Anchor mierzy RTD_resp swoim zegarem — jest rozciągnięty o czynnik (1+ε).
 * clockOffsetRatio ≈ ε, odczytywany przez DW3000 z carrier integratora.
 * Bez korekcji błąd przy 800 µs opóźnieniu = ~2.4 m (dla ε = 20 PPM).
 *
 * Dlaczego uśredniamy: jeden pomiar ma szum ~±10 cm (multipath, termiczny).
 * Uśrednienie N próbek zmniejsza szum o sqrt(N) — przy N=10 ok. 3x.
 * Odchylenie standardowe (stddev) informuje o stabilności środowiska.
 */

#define APP_NAME "SS TWR AVERAGED v1.0"

// Piny SPI do DW3000
const uint8_t PIN_RST = D3; // reset — aktywny niski, inicjalizuje chip
const uint8_t PIN_IRQ = D0; // przerwanie — DW3000 sygnalizuje gotowość
const uint8_t PIN_SS  = D7; // chip select SPI

/*
 * Konfiguracja fizyczna UWB.
 * Obie strony (tag i anchor) MUSZĄ mieć identyczną konfigurację,
 * bo inaczej nie będą się słyszeć.
 */
static dwt_config_t config = {
    5,               // kanał 5: 6240–6739 MHz — najlepszy zasięg, legalny globalnie
    DWT_PLEN_128,    // preambuła 128 symboli — kompromis zasięg/czas nadawania
    DWT_PAC8,        // PAC8 — okno akwizycji preambuły, pasuje do PLEN_128
    9,               // kod preambuły TX = 9 (dla kanału 5, musi zgadzać się z RX)
    9,               // kod preambuły RX = 9
    1,               // niestandardowy SFD 8-symbol — krótszy niż standard, szybszy
    DWT_BR_6M8,      // 6.8 Mbps — najszybszy tryb, minimalizuje RTD_resp
    DWT_PHRMODE_STD, // standardowy nagłówek PHY
    DWT_PHRRATE_STD, // standardowa szybkość nagłówka
    (129 + 8 - 8),   // SFD timeout = preambuła + 1 + SFD - PAC; okno na synchronizację
    DWT_STS_MODE_OFF,// STS wyłączone — nie potrzebujemy bezpiecznego timestampu
    DWT_STS_LEN_64,  // długość STS (nieaktywne, ale wymagane przez strukturę)
    DWT_PDOA_M0      // Phase Difference of Arrival wyłączone — jeden tag, jedna antena
};

/*
 * Opóźnienie anteny: czas między "wysłaniem" bitu przez chip a faktyczną emisją RF.
 * Wartość 16395 DTU ≈ 256.6 ns — typowa dla DW3000 z zewnętrzną anteną.
 * Dodawana do każdego timestampu TX i odejmowana od RX, żeby timestamps
 * reprezentowały moment przejścia fali przez antenę, nie moment w SPI.
 * Wymaga kalibracji per-urządzenie dla dokładności < 5 cm.
 */
#define TX_ANT_DLY 16395  // jednostki DTU (1 DTU ≈ 15.65 ps)
#define RX_ANT_DLY 16395

/*
 * Ile czasu po wysłaniu Poll tag czeka zanim włączy odbiornik.
 * Musi być trochę krótsze niż POLL_RX_TO_RESP_TX_DLY_UUS anchora
 * minus czas transmisji ramki Response — żeby okno RX trafiło w transmisję.
 * Zmienione z 240 na 500 µs bo ESP32-C3 jest wolniejszy niż STM32/RPi.
 */
#define POLL_TX_TO_RESP_RX_DLY_UUS 500

/*
 * Timeout odbioru Response od momentu włączenia odbiornika.
 * Musi być dłuższy niż czas transmisji ramki Response (~200 µs przy 6.8 Mbps).
 * Zmienione z 400 na 800 µs — zapas na jitter i wolniejszy MCU.
 */
#define RESP_RX_TIMEOUT_UUS 800

// Indeksy i stałe dla struktury ramek IEEE 802.15.4
#define ALL_MSG_COMMON_LEN  10  // wspólne pierwsze 10 bajtów obu ramek (frame ctrl, PAN, adresy, kod)
#define ALL_MSG_SN_IDX       2  // bajt 2: numer sekwencji (ignorowany przy weryfikacji)
#define RESP_MSG_POLL_RX_TS_IDX 10  // bajty 10–13 w Response: timestamp odbioru Poll przez anchor (T2)
#define RESP_MSG_RESP_TX_TS_IDX 14  // bajty 14–17 w Response: timestamp nadania Response przez anchor (T3)
#define RESP_MSG_TS_LEN      4
#define RX_BUF_LEN          20

/*
 * Parametry serii uśredniania.
 * AVG_SAMPLES: więcej = mniejszy szum, ale wolniejszy odczyt
 * SAMPLE_DELAY_MS: przerwa między próbkami; zbyt mała może powodować
 *   kolizje jeśli anchor nie zdąży wrócić do nasłuchu
 */
#define AVG_SAMPLES      10
#define SAMPLE_DELAY_MS  50
#define SERIES_DELAY_MS  2000

/*
 * Ramki zgodne z IEEE 802.15.4 (Decawave-specific ranging frames).
 * Bajty [5–8] to adresy 'WAVE'/'VEWE' — hardcoded, w produkcji każde
 * urządzenie powinno mieć unikalny adres żeby uniknąć kolizji odpowiedzi.
 * Bajt [9] = 0xE0 (Poll) / 0xE1 (Response) — kod funkcji rangingu.
 * Ostatnie 2 bajty to CRC, automatycznie wstawiane przez DW3000.
 *   bajt:  0     1    2    3     4    5    6    7    8    9   10  11
 */
static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint8_t  frame_seq_nb = 0;    // numer sekwencji: inkrementowany po każdym TX
static uint8_t  rx_buffer[RX_BUF_LEN];
static uint32_t status_reg = 0;

extern dwt_txconfig_t txconfig_options; // konfiguracja mocy TX z dw3000_config_options.cpp

/*
 * Wykonuje jeden pomiar SS-TWR i zapisuje wynik do *out_distance.
 * Zwraca false jeśli odpowiedź nie nadeszła (timeout/błąd CRC).
 * Odrzucone próbki nie wchodzą do uśrednienia — dzięki temu
 * multipath outliers nie psują średniej.
 */
static bool single_measurement(double *out_distance) {
    // Wstaw numer sekwencji i nadaj Poll (T1 rejestrowane sprzętowo przez DW3000)
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK); // wyczyść flagę TX done
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); // ostatni arg=1: to ramka rangingowa (ustawia RMARKER)
    // DWT_RESPONSE_EXPECTED: po TX chip automatycznie włącza RX po POLL_TX_TO_RESP_RX_DLY_UUS
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    // Busy-wait na status: czekaj na dobrą ramkę LUB timeout LUB błąd
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {};

    frame_seq_nb++; // inkrementuj zawsze, żeby anchor mógł wykryć zagubione ramki

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
        // Timeout lub błąd CRC — anchor nie odpowiedział lub ramka uszkodzona
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return false;
    }

    // Dobra ramka — odczytaj długość i treść
    uint32_t frame_len;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK; // RXFLEN_MASK: tylko bity długości
    if (frame_len > sizeof(rx_buffer)) return false;

    dwt_readrxdata(rx_buffer, frame_len, 0);
    rx_buffer[ALL_MSG_SN_IDX] = 0; // wyzeruj SN przed porównaniem (SN nie jest weryfikowany)

    // Sprawdź czy to odpowiedź od naszego anchora (pierwsze 10 bajtów muszą się zgadzać)
    if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) != 0) return false;

    // --- Odczyt czterech timestampów do obliczenia TOF ---

    // T1 i T4: timestampy z rejestrów DW3000 tego chipa (zegar tagu)
    uint32_t poll_tx_ts = dwt_readtxtimestamplo32(); // T1: moment nadania Poll
    uint32_t resp_rx_ts = dwt_readrxtimestamplo32(); // T4: moment odbioru Response
    // Używamy tylko 32 bitów z 40-bitowego timestampa — bezpieczne bo T4-T1 < 67 ms (2^32 DTU)

    // clockOffsetRatio: różnica częstotliwości zegarów tag/anchor, mierzona sprzętowo.
    // DW3000 carrier integrator śledzi fazę nośnej — jej dryft = dryft zegara nadajnika.
    // Wynik jest 26-bitową liczbą ze znakiem, normalizujemy do zakresu (-0.5, +0.5).
    float clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

    // T2 i T3: timestampy anchora, przesłane wewnątrz ramki Response (bajty 10–17)
    uint32_t poll_rx_ts, resp_tx_ts;
    resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts); // T2
    resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts); // T3

    // RTD_init = T4 - T1 (round-trip delay mierzony przez tag, jego zegar)
    // RTD_resp = T3 - T2 (czas odpowiedzi mierzony przez anchor, jego zegar)
    int32_t rtd_init = resp_rx_ts - poll_tx_ts;
    int32_t rtd_resp = resp_tx_ts - poll_rx_ts;

    // Korekcja dryfu: RTD_resp jest w zegarze anchora który biegnie z offsetem ε.
    // Mnożymy przez (1 - clockOffsetRatio) żeby przeliczyć na czas tagu.
    // DWT_TIME_UNITS = 1/(128×499.2 MHz) ≈ 15.65 ps — przelicznik DTU → sekundy
    double tof = ((rtd_init - rtd_resp * (1.0 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
    *out_distance = tof * SPEED_OF_LIGHT;
    return true;
}

void setup() {
    UART_init();
    test_run_info((unsigned char *)APP_NAME);

    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(PIN_SS);
    delay(2); // DW3000 potrzebuje ~2 ms na przejście INIT_RC → IDLE_RC po resecie

    while (!dwt_checkidlerc()) { UART_puts("IDLE FAILED\r\n"); while (1); }
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) { UART_puts("INIT FAILED\r\n"); while (1); }

    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK); // LED miga przy każdym TX/RX — przydatne do debugowania

    if (dwt_configure(&config)) { UART_puts("CONFIG FAILED\r\n"); while (1); }

    dwt_configuretxrf(&txconfig_options);        // moc i kształt widma TX
    dwt_setrxantennadelay(RX_ANT_DLY);           // kompensacja opóźnienia anteny przy odbiorze
    dwt_settxantennadelay(TX_ANT_DLY);           // kompensacja opóźnienia anteny przy nadaniu
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS); // kiedy włączyć RX po zakończeniu TX
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);       // jak długo czekać na Response zanim zgłosi timeout
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);   // włącz wzmacniacz LNA (RX) i PA (TX)
}

void loop() {
    double samples[AVG_SAMPLES];
    int    valid = 0;

    // Zbierz AVG_SAMPLES próbek — odrzucone (timeout/błąd) nie wchodzą do tablicy
    for (int i = 0; i < AVG_SAMPLES; i++) {
        double d;
        if (single_measurement(&d)) {
            samples[valid++] = d;
        }
        Sleep(SAMPLE_DELAY_MS); // przerwa żeby anchor zdążył wrócić do nasłuchu
    }

    if (valid == 0) {
        Serial.println("No valid samples in this series");
        Sleep(SERIES_DELAY_MS);
        return;
    }

    // Średnia arytmetyczna: redukuje szum losowy o czynnik sqrt(valid)
    double sum = 0;
    for (int i = 0; i < valid; i++) sum += samples[i];
    double mean = sum / valid;

    // Odchylenie standardowe (populacyjne): miara rozrzutu pomiarów.
    // Duże stddev → dużo multipathów lub przeszkody między tagiem a anchorem.
    // Typowe wartości w LOS: ~1–3 cm, w NLOS: >10 cm.
    double var = 0;
    for (int i = 0; i < valid; i++) var += (samples[i] - mean) * (samples[i] - mean);
    double stddev = sqrt(var / valid);

    Serial.print("Samples: ");  Serial.print(valid);
    Serial.print("  Mean: ");   Serial.print(mean, 4);
    Serial.print(" m  Stddev: "); Serial.print(stddev * 100.0, 2);
    Serial.println(" cm");

    Sleep(SERIES_DELAY_MS);
}
