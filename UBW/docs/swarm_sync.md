# Synchronizacja i lokalizacja roju bez stałych anchorów

> Problem: N pojazdów musi znać swoje wzajemne pozycje bez żadnej
> zewnętrznej infrastruktury — bez GPS, bez stałych anchorów.

---

## Dlaczego to jest trudne

W klasycznym systemie UWB:
- Anchory stoją w znanych miejscach → dostarczają układ odniesienia
- Tagi pytają anchory → dostają swoją pozycję

W roju:
- Nie ma niczego "stałego" → układ odniesienia musi być zbudowany z pomiarów między pojazdami
- Pojazdy się poruszają → pozycje zmieniają się szybciej niż można zmierzyć
- Każde dwa pojazdy potrzebują wymiany ramek → N pojazdów = N(N-1)/2 par = O(N²) pomiarów
- Wszyscy nadają na tym samym medium → kolizje bez koordynacji

---

## Trzy warstwy problemu

```
┌──────────────────────────────────────────┐
│  3. LOKALIZACJA WZGLĘDNA                 │  "Gdzie jest każdy?"
│     MDS / trilateration / filter         │
├──────────────────────────────────────────┤
│  2. RANGING — macierz odległości         │  "Jak daleko parami?"
│     DS-TWR parami + TDMA scheduling      │
├──────────────────────────────────────────┤
│  1. SYNCHRONIZACJA ZEGARÓW               │  "Kiedy jest czyja kolej?"
│     TWR-based clock offset estimation    │
└──────────────────────────────────────────┘
```

Każda warstwa buduje na poprzedniej. Poniżej — szczegóły każdej.

---

## Warstwa 1 — Synchronizacja zegarów

### Po co w ogóle synchronizować?

Przy DS-TWR i SS-TWR parami synchronizacja **nie jest potrzebna** — algorithmy są odporne na dryft (patrz `ranging_algorithms.md`).

Synchronizacja jest potrzebna gdy:
1. Chcesz używać TDoA (wymaga wspólnej osi czasu między listenerami)
2. Chcesz precyzyjnie planować sloty TDMA (bez synca sloty się rozjeżdżają)
3. Chcesz korelować pomiary z różnych pojazdów (ten sam "czas globalny")

### Metoda 1 — TWR-based clock offset estimation

Używając SS-TWR lub DS-TWR możemy wyestymować nie tylko odległość, ale też **offset i dryft zegara** jednego pojazdu względem drugiego.

Ze wzoru SS-TWR (bez korekcji dryfu):
```
TOF_zmierzony = TOF_prawdziwy + ε × RTD_resp / 2
```

Jeśli znamy TOF z poprzedniego pomiaru (np. pojazdy stoją), możemy obliczyć ε:
```
ε = 2 × (TOF_zmierzony - TOF_prawdziwy) / RTD_resp
```

Lub bardziej bezpośrednio — ze wzoru SS-TWR z korekcją:
```
clockOffsetRatio = ((float)dwt_readclockoffset()) / (1 << 26)
```

To jest offset zegara nadajnika (anchora) względem odbiornika (tagu). Po każdym pomiarze TWR tag wie jak bardzo zegar anchora biegnie szybciej/wolniej.

**Protokół synchronizacji TWR:**
```
Co SYNC_INTERVAL_MS (np. 500 ms):
  1. Pojazd A robi SS-TWR z pojazdem B
  2. Odczytuje clockOffsetRatio (= ε_B względem A)
  3. Zapamiętuję: clock_offset_B = clock_offset_B × α + ε_B × (1-α)
     (filtr exponencjalny, α ≈ 0.8 — tłumi szum pomiaru)
  4. Aby przeliczyć timestamp z B na oś czasu A:
     ts_A = ts_B × (1 + clock_offset_B)
```

Dokładność: ~10–50 ns po filtracji, zależy od stabilności kryształu (±20 PPM typowe).

### Metoda 2 — Reference Broadcast Synchronization (RBS) przez UWB

Jeden pojazd ("master sync") nadaje broadcast ramkę sync co SYNC_PERIOD.
Pozostałe pojazdy odbierają i zapisują timestamp odbioru.

```
Master:    |──Sync TX──|                     |──Sync TX──|
Slave 1:              |──T1_new──|                       |──T1_new──|
Slave 2:              |──T2_new──|                       |──T2_new──|

Slave 1 oblicza swój dryft:
  drift_1 = (T1_new - T1_old) / SYNC_PERIOD_expected_DTU - 1.0
  offset_1 += drift_1 * elapsed_time
```

Zaletą RBS jest że propagacja sygnału jest ta sama do wszystkich slavów (broadcast) — nie potrzeba wiedzieć odległości do synca.

Wada: wymaga wyznaczenia jednego pojazdu jako mastera.

### Metoda 3 — Flooding Time Synchronization (FTSP) dla UWB

Algorytm rozproszony — żaden pojazd nie jest na stałe masterem. Pojazd z najniższym ID jest masterem do chwili gdy wypadnie z zasięgu.

Każdy pojazd rozsyła swój "czas globalny" wraz z lokalnym timestampem wysłania.
Odbiorca porównuje odebrany "czas globalny" z własnym lokalnym:
```
offset = received_global_time - local_time_at_receipt
rate   = (offset - previous_offset) / elapsed_local_time
```

FTSP jest odporny na wpadnięcia pojazdów, ale zbieżność trwa kilka cykli wymiany.

---

## Warstwa 2 — Ranging parami (macierz odległości)

### Problem kolizji — TDMA

Gdy N pojazdów chce jednocześnie mierzyć odległości, ramki kolidują.
Rozwiązanie: każdy pojazd dostaje własny **slot czasowy**.

```
TDMA z N=4 pojazdami, slot = 10 ms:

t=0ms    t=10ms   t=20ms   t=30ms   t=40ms   t=50ms...
│ V1 ───│ V2 ───│ V3 ───│ V4 ───│ V1 ───│ V2 ...
 ↑        ↑        ↑        ↑
 V1       V2       V3       V4
 robi     robi     robi     robi
 DS-TWR   DS-TWR   DS-TWR   DS-TWR
 z V2,V3  z V1,V3  z V1,V2  z V1,V2
 V4       V4
```

W swoim slocie pojazd Vₙ po kolei odpytuje każdy inny pojazd DS-TWR.
Pozostałe pojazdy w tym czasie są w trybie RX (anchor/responder).

### Przypisanie slotów

Najprostsza metoda: slot_id = ID pojazdu, slot_duration = MAX_PAIRS × RTT_DS-TWR.

```
RTT_DS-TWR ≈ POLL_RX_TO_RESP_TX + RESP_RX_TO_FINAL_TX + czas transmisji × 3
           ≈ 800 + 1000 + 3×200 µs ≈ 2.4 ms na parę

Dla N=5: każdy pojazd mierzy 4 pary × 2.4 ms = 9.6 ms / slot
Cykl = 5 × 9.6 ms = 48 ms → update rate ≈ 20 Hz
```

Przy 20 Hz możliwa lokalizacja szybko poruszających się obiektów (do ~5 m/s z marginesem).

### Inicjalizacja bez centralnego koordynatora

Problem kurczaka i jajka: żeby nadawać w swoim slocie trzeba wiedzieć ile jest pojazdów, ale żeby to wiedzieć trzeba się komunikować.

Rozwiązanie — **beacon + join protokół**:

```
1. Nowy pojazd nadaje BeaconRequest (broadcast)
2. Istniejące pojazdy odpowiadają BeaconResponse ze swoim ID i listą znanych pojazdów
3. Nowy pojazd wybiera wolne ID (najniższe nieużywane)
4. Nadaje JoinAnnounce z nowym ID
5. Wszyscy aktualizują N i przeliczają długości slotów
```

Prosty wariant bez BeaconRequest: pojazd nasłuchuje przez 2×MAX_SLOT_TIME.
Jeśli nikt nie mówił → jest pierwszy, dostaje ID=0. Jeśli ktoś mówił → dostaje max_id+1.

### Protokół ranging w slocie (pseudokod)

```cpp
// W slocie pojazdu o ID = my_id:

void ranging_slot() {
    for (int target_id = 0; target_id < num_vehicles; target_id++) {
        if (target_id == my_id) continue;

        double distance = ds_twr_measure(target_id);
        if (distance > 0) {
            distance_matrix[my_id][target_id] = distance;
            distance_matrix[target_id][my_id] = distance; // symetryczna
        }

        // Krótki odstęp między parami żeby target zdążył wrócić do RX
        delay_us(500);
    }
}

// Poza swoim slotem: tryb RX (responder dla DS-TWR)
void idle_slot() {
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    // Odpowiadaj na DS-TWR Poll z dowolnego pojazdu
    // Kiedy odebrany Final: raportuj wynik przez internal bus
}
```

---

## Warstwa 3 — Lokalizacja względna

Mając macierz odległości `D[i][j]` między wszystkimi parami pojazdów, chcemy obliczyć ich wzajemne pozycje w 2D lub 3D.

### Metoda A — MDS (Multidimensional Scaling)

MDS to klasyczna metoda rekonstrukcji pozycji z macierzy odległości.

```
Wejście: macierz D[N×N] odległości euklidesowych
Wyjście: macierze X[N×2] lub X[N×3] — pozycje pojazdów

Algorytm (klasyczny MDS):
1. B = -0.5 × H × D² × H     (gdzie H = I - (1/N)×11ᵀ to "centering matrix")
2. [V, Λ] = eigen(B)           (rozkład własny)
3. X = V_k × sqrt(Λ_k)        (k = 2 dla 2D, k = 3 dla 3D)
```

Układ współrzędnych jest względny i nie ma orientacji — wymaga dodatkowego ograniczenia (np. jeden pojazd = origin, inny definiuje oś X).

Implementacja w C++ (na pokładzie):
- `B` oblicza się przez prostą algebrę macierzową
- Wartości własne można obliczać iteracyjnie metodą potęgową (dla małych N < 10 wystarczy ~50 iteracji)
- Biblioteki Eigen lub własna implementacja dla ESP32

### Metoda B — Trilateration (dla N ≥ 3)

Jeśli znasz pozycje 3 pojazdów (np. z poprzedniego kroku MDS lub z GPS), nowy czwarty można lokalizować jako:

```
d₁ = odległość od V4 do V1 (zmierzone)
d₂ = odległość od V4 do V2 (zmierzone)
d₃ = odległość od V4 do V3 (zmierzone)

Układ równań:
  (x - x₁)² + (y - y₁)² = d₁²
  (x - x₂)² + (y - y₂)² = d₂²
  (x - x₃)² + (y - y₃)² = d₃²

Rozwiązanie przez linearyzację (odejmij 1. od 2. i 3.):
  2(x₂-x₁)x + 2(y₂-y₁)y = d₁²-d₂² + x₂²-x₁² + y₂²-y₁²
  2(x₃-x₁)x + 2(y₃-y₁)y = d₁²-d₃² + x₃²-x₁² + y₃²-y₁²
  → Ax = b → x = A⁻¹ b
```

### Metoda C — Extended Kalman Filter (EKF)

Dla poruszających się pojazdów, MDS/trilateration daje skokowe pozycje (tylko z pomiaru). EKF wygładza trajektorię:

```
Model ruchu (state = [x, y, vx, vy]):
  x(t+1) = x(t) + vx×dt
  y(t+1) = y(t) + vy×dt
  vx(t+1) = vx(t) + ax×dt    (ax z IMU lub = 0)
  vy(t+1) = vy(t) + ay×dt

Model obserwacji:
  d_ij = sqrt((x_i - x_j)² + (y_i - y_j)²) + szum

EKF łączy predykcję z modelu ruchu z pomiarami UWB
→ gładka trajektoria + odporność na missing measurements
```

Dla roju dronów: każdy dron uruchamia własny EKF, obserwacjami są odległości do pozostałych dronów. Pozycje pozostałych dronów (z ich własnych EKF) można wymieniać przez UWB lub WiFi.

---

## Architektura pełnego systemu roju

```
┌─────────────────────────────────────────────────────────────┐
│ Każdy pojazd uruchamia ten sam stack:                        │
│                                                              │
│  ┌──────────┐  ┌──────────────┐  ┌──────────────────────┐   │
│  │ DW3000   │  │  TDMA        │  │  Localization        │   │
│  │ driver   │→ │  scheduler   │→ │  engine              │   │
│  │          │  │  (slot mgmt) │  │  MDS + EKF           │   │
│  └──────────┘  └──────────────┘  └──────────────────────┘   │
│       ↕               ↕                     ↕                │
│  ┌──────────┐  ┌──────────────┐  ┌──────────────────────┐   │
│  │ Clock    │  │  Ranging     │  │  State broadcast     │   │
│  │ sync     │  │  DS-TWR      │  │  (pozycja → sąsiedzi)│   │
│  │ module   │  │  pairs       │  │                      │   │
│  └──────────┘  └──────────────┘  └──────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

Wymiana pozycji (State broadcast) może być przez:
- UWB data frames (wbudowane w ramki TDMA)
- WiFi/ESP-NOW (równolegle — ESP32-C3 ma WiFi)
- LoRa (większy zasięg)

---

## Ograniczenia i pułapki

### Liczba pojazdów vs update rate

```
N pojazdów:       2    4    8    16
Pary pomiarów:    1    6   28   120
Czas cyklu [ms]: 2.4  14   67   288
Update rate [Hz]: 400  70   15    3.5
```

Przy N > 8 update rate spada poniżej 10 Hz — za wolno dla szybko manewrujących dronów. Rozwiązanie: nie mierz wszystkich par w każdym cyklu — priorytetyzuj bliskich sąsiadów.

### NLOS i przeszkody

Pojazdy mogą się zasłaniać (własne ciało drona, inne drony między nimi). NLOS powoduje zawyżone odległości (sygnał obchodzi przeszkodę). Filtrowanie: odrzucaj pomiary z niską jakością sygnału (CIR peak analysis) lub używaj metod odpornych na outliers (median zamiast mean).

### Inicjalizacja orientacji

MDS daje pozycje względne bez orientacji bezwzględnej. Rój może wiedzieć że "V2 jest 3 m od V1" ale nie "na wschód". Do orientacji potrzeba:
- Magnetometr (każdy pojazd)
- GPS (choćby jeden pojazd w roju)
- Znany punkt odniesienia (start z wyznaczonej pozycji)

### Propagacja błędu

Błędy pomiarów odległości propagują się do pozycji przez DOP (patrz ranging_algorithms.md). W roju DOP zależy od geometrii — drony latające w linii mają wysoki DOP, drony w formacji 2D mają niski.

---

## Proponowana kolejność implementacji

1. **Faza 1 (już zrobione):** SS-TWR i DS-TWR parami — walidacja hardware
2. **Faza 2:** TDMA scheduler + DS-TWR dla N=3–4 pojazdów
3. **Faza 3:** Clock sync (clockOffsetRatio z DS-TWR)
4. **Faza 4:** MDS dla pozycji względnych (offline na PC)
5. **Faza 5:** EKF na pokładzie ESP32-C3
6. **Faza 6:** State broadcast (pozycje przez UWB lub WiFi)
7. **Faza 7:** TDoA jako alternatywa dla dużych rojów (N > 10)
