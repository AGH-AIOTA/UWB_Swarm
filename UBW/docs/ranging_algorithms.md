# Algorytmy rangingu UWB

> Implementacje: `src/ss_twr/`, `src/ds_twr/`, `src/tdoa/`
> Synchronizacja roju bez anchorów: `docs/swarm_sync.md`

---

## Podstawy — jednostki czasu DW3000

Zanim zaczniesz czytać wzory, warto znać jednostki:

| Symbol | Wartość | Znaczenie |
|---|---|---|
| 1 DTU | ≈ 15.65 ps | Device Time Unit (1 / (128 × 499.2 MHz)) |
| 1 UUS | = 65536 DTU ≈ 1.0256 µs | UWB Microsecond |
| Timestamp | 40 bitów | przepełnienie co ~17.2 s |
| `DWT_TIME_UNITS` | = 1/(128×499.2e6) | stała w bibliotece, przelicznik DTU→s |

Gdy kod używa `lo32` timestampa, odrzuca górny bajt. Bezpieczne dopóki różnica < 2³² DTU ≈ 67 ms.

---

## 1. SS-TWR — Single-Sided Two-Way Ranging

### Schemat wymiany

```
         T1          T4
Tag:    ──●────────────●──────────────
          │  Poll TX   │  Resp RX
          │            │
Anchor:   │   T2       │   T3
        ──────●────────────●──────────
               Poll RX     Resp TX
```

### Cztery timestampy i co znaczą

| Symbol | Kto mierzy | Co to jest |
|---|---|---|
| T1 | Tag (zegar A) | moment nadania Poll |
| T2 | Anchor (zegar B) | moment odbioru Poll |
| T3 | Anchor (zegar B) | moment nadania Response |
| T4 | Tag (zegar A) | moment odbioru Response |

### Podstawowa formuła (bez korekcji)

```
RTD_init = T4 - T1       (round-trip mierzony przez tag, zegar A)
RTD_resp = T3 - T2       (czas odpowiedzi anchora, zegar B)

TOF = (RTD_init - RTD_resp) / 2
```

**Problem:** T3 i T2 mierzone są zegarem B, który biegnie z offsetem `ε` względem A. Z perspektywy tagu `RTD_resp` jest rozciągnięty:

```
RTD_resp_zmierzone = RTD_resp_prawdziwe × (1 + ε)
```

Błąd odległości:
```
Δd = ε × RTD_resp × c / 2
   = 20×10⁻⁶ × 800×10⁻⁶ × 3×10⁸ / 2 ≈ 2.4 m   (dla ε = 20 PPM)
```

### Korekcja przez carrier integrator

DW3000 mierzy `ε` sprzętowo — carrier integrator porównuje oczekiwaną częstotliwość nośnej z odebraną. Ponieważ ten sam oscylator generuje i zegar i nośną, różnica częstotliwości = dryft zegara.

```
TOF = (RTD_init - RTD_resp × (1 - ε)) / 2
```

Wyprowadzenie: skrócić `(1+ε)` przez podzielenie RTD_resp przez `(1+ε) ≈ (1-ε)` dla małych ε.

```cpp
// W kodzie:
clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);
// Rejestr 26-bit ze znakiem → normalizacja do (-0.5, +0.5) = frakcja częstotliwości
// Typowe wartości: ±5 do ±30 PPM zależnie od jakości kryształu

tof = ((rtd_init - rtd_resp * (1.0 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
```

### Timing okien RX/TX (diagram szczegółowy)

```
Tag TX Poll zakończony
│
│←── POLL_TX_TO_RESP_RX_DLY_UUS ──→│← okno RX →│
│         (500 µs)                   │ TIMEOUT    │
│                                    │ (800 µs)   │
                                     │            │
Anchor RX Poll
│←── POLL_RX_TO_RESP_TX_DLY_UUS ──→│ TX Response│
│         (800 µs)                   │            │
```

`POLL_TX_TO_RESP_RX_DLY_UUS` musi być krótszy niż czas od wysłania Poll do nadejścia Response — czyli mniej niż `POLL_RX_TO_RESP_TX_DLY_UUS + 2×TOF`. Okno RX (`RESP_RX_TIMEOUT_UUS`) musi być szersze niż czas transmisji ramki Response (~200 µs przy 6.8 Mbps).

### Parametry i dokładność

| Parametr | Wartość | Wpływ na dokładność |
|---|---|---|
| Opóźnienie odpowiedzi | 800 µs | Im krótsze, tym mniejszy błąd dryfu |
| Data rate | 6.8 Mbps | Krótka ramka = krótkie opóźnienie |
| Kalibracja anteny | TX/RX ANT DLY | Błąd stały ~1 cm na 0.1 ns odchyłki |
| Multipath | środowisko | Dominujący błąd w NLOS: 10–100 cm |

| | |
|---|---|
| Liczba ramek | 2 |
| Dokładność (LOS, z korekcją) | ~5–15 cm |
| Dokładność (bez korekcji) | ~1–3 m |
| Wymaga synchronizacji | nie |
| Skalowalność tagów | mała (1 tag na raz) |

**Implementacja:** `src/ss_twr/` (basic, averaged, no_correction)

---

## 2. DS-TWR — Double-Sided Two-Way Ranging

### Schemat wymiany

```
         T1                T4        T5
Tag:    ──●────────────────●──────────●────────
          │ Poll TX         │ Resp RX  │ Final TX
          │                 │          │
Anchor:   │  T2        T3   │          │  T6
        ────●────────────●─────────────────●───
              Poll RX    Resp TX            Final RX
```

### Sześć timestampów — dwa zegarów

| Symbol | Kto mierzy | Zegar |
|---|---|---|
| T1 | Tag | A |
| T4 | Tag | A |
| T5 | Tag | A |
| T2 | Anchor | B |
| T3 | Anchor | B |
| T6 | Anchor | B |

```
Ra = T4 - T1   (round-trip tagu,      zegar A)
Da = T5 - T4   (czas odpow. tagu,     zegar A)
Db = T3 - T2   (czas odpow. anchora,  zegar B)
Rb = T6 - T3   (round-trip anchora,   zegar B)
```

### Formuła DS-TWR i dlaczego eliminuje dryft

```
         Ra × Rb  −  Da × Db
TOF =  ─────────────────────────
         Ra + Rb  +  Da + Db
```

**Wyprowadzenie (uproszczone):**

Jeśli zegar B biegnie z offsetem ε względem A, to Rb i Db są zawyżone o czynnik (1+ε):
```
Rb_mierzone = Rb_prawdziwe × (1+ε)
Db_mierzone = Db_prawdziwe × (1+ε)
```

W liczniku `Ra×Rb − Da×Db` oba iloczyny zawierają `(1+ε)`:
```
(Ra × Rb_pr.(1+ε)) − (Da × Db_pr.(1+ε)) = (1+ε) × (Ra×Rb_pr. − Da×Db_pr.)
```

W mianowniku `Ra + Rb + Da + Db`:
```
Ra + Rb_pr.(1+ε) + Da + Db_pr.(1+ε) = (Ra + Da) + (1+ε)(Rb_pr. + Db_pr.)
                                     ≈ (1+ε/2) × (Ra + Rb_pr. + Da + Db_pr.)
```

Czynnik `(1+ε)` skraca się do rzędu ε², więc błąd wynosi:
```
Δd_DS-TWR ≈ ε² × T_reply × c / 2
           = (20×10⁻⁶)² × 800×10⁻⁶ × 3×10⁸ / 2
           ≈ 48 µm   (vs 2.4 m w SS-TWR bez korekcji)
```

DS-TWR **nie potrzebuje pomiaru dryfu** — błąd jest z natury drugiego rzędu.

### Jak anchor dostaje Ra i Da

Tag wpisuje Ra i Da do treści ramki Final (bajty 10–13 i 14–17). Anchor po odebraniu Final ma wszystkie 4 wartości:
- Ra, Da — z payload Final
- Db = T3 - T2 — lokalnie
- Rb = T6 - T3 — lokalnie

Oblicza TOF i drukuje dystans. W wersji 4-ramkowej anchor odsyła wynik w ramce Report.

| | |
|---|---|
| Liczba ramek | 3 (anchor oblicza) lub 4 (tag oblicza) |
| Dokładność | ~2–10 cm |
| Wymaga synchronizacji | nie |
| Skalowalność tagów | mała |

**Implementacja:** `src/ds_twr/`

---

## 3. ADS-TWR — Asymmetric DS-TWR

Wariant DS-TWR gdzie celowo dobiera się `Da = Db` (symetryczne opóźnienia). Wtedy formuła upraszcza się do:

```
TOF = (Ra + Rb - Da - Db) / 4
```

Błąd dryfu zeruje się dokładnie (nie tylko do ε²) przy `Da = Db`. W praktyce Da ≈ Db ≈ 1000 µs — wystarczy ustawić oba opóźnienia na tę samą wartość.

| | |
|---|---|
| Dokładność | ~2–5 cm (przy dobrze dobranych opóźnieniach) |
| Trudność | dobór równych opóźnień Da = Db na różnych platformach |

---

## 4. TDoA — Time Difference of Arrival

### Idea

Tag nadaje **jeden blink**. Wiele listenerów odbiera go w różnych chwilach — różnice czasów odbioru definiują hiperbole, których przecięcie to pozycja tagu.

```
                        [Tag]
                       / | \
                τ₁  /   |   \  τ₃
                  /     |τ₂   \
                /       |       \
         [L1]          [L2]         [L3]
          T1            T2           T3

TDoA₁₂ = T1 - T2 = (τ₁ - τ₂) × c  →  hiperbola z ogniskami L1, L2
TDoA₁₃ = T1 - T3 = (τ₁ - τ₃) × c  →  hiperbola z ogniskami L1, L3

Pozycja tagu = przecięcie hiperboli
```

### Geometria — hiperbole

Dla listenerów L1 i L2 w znanych pozycjach, TDoA₁₂ definiuje hipetbolę: zbiór punktów gdzie różnica odległości do L1 i L2 wynosi `TDoA₁₂ × c`. W 2D dwie hiperbole dają 2 rozwiązania (jedno eliminuje się geometrycznie). W 3D potrzeba 4 listenerów.

```
Δd₁₂ = TDoA₁₂ × c = d(Tag,L1) - d(Tag,L2)
Δd₁₃ = TDoA₁₃ × c = d(Tag,L1) - d(Tag,L3)

Minimalizacja metodą NLLS (Nonlinear Least Squares):
  x̂ = argmin Σᵢⱼ (Δdᵢⱼ_zmierzone - Δdᵢⱼ_obliczone(x))²
```

### Synchronizacja listenerów — kluczowy problem

Listenerzy muszą mierzyć czas w **tej samej skali** — inaczej TDoA zawiera błąd dryfu:
```
Błąd TDoA = ε × T_między_blinkami
```
Przy ε = 20 PPM i 1 s między blinkami: błąd = 20 µs → 6 m pozycji!

**Rozwiązania:**
1. **TWR-based sync (preferowane):** master anchor wysyła co chwilę ramkę sync. Każdy listener robi SS-TWR z masterem i oblicza swój offset zegara.
2. **Cable sync:** kabel trigger do wszystkich listenerów (tylko w laboratorium).
3. **Cooperative sync (rój bez anchorów):** opisane w `docs/swarm_sync.md`.

### Format ramek TDoA

```
Blink (Tag → wszystkie):
  bajt 0-1:  frame control
  bajt 2:    numer sekwencji
  bajt 3-4:  PAN ID
  bajt 5-6:  adres tagu (unikalny!)
  bajt 7:    kod funkcji 0xE3 (Blink)
  bajty 8-9: CRC (auto)

Report (Listener → UART/sieć):
  "TDOA,<tag_id>,<seq>,<listener_id>,<timestamp_lo32>\n"
```

### Obliczenie pozycji (na PC/RPi)

Zbierz raporty ze wszystkich listenerów dla tego samego blink (ten sam `seq`). Mając N listenerów i ich pozycje `(xᵢ, yᵢ)`:

```python
# Pseudo-kod NLLS:
from scipy.optimize import minimize

def residuals(pos, listeners, tdoa_measurements):
    x, y = pos
    errors = []
    for (i, j), tdoa_ij in tdoa_measurements:
        di = sqrt((x - listeners[i].x)² + (y - listeners[i].y)²)
        dj = sqrt((x - listeners[j].x)² + (y - listeners[j].y)²)
        errors.append((di - dj) - tdoa_ij * c)
    return sum(e² for e in errors)

result = minimize(residuals, x0=[0, 0], ...)
```

| | |
|---|---|
| Liczba ramek (tag) | 1 |
| Min. listenerów (2D) | 3 |
| Min. listenerów (3D) | 4 |
| Dokładność | ~10–30 cm (zależy od geometrii i sync) |
| Wymaga synchronizacji listenerów | **tak** |
| Skalowalność tagów | **bardzo duża** — N tagów jednocześnie |
| Update rate | wysoki — tylko 1 ramka na pomiar |

**Implementacja:** `src/tdoa/` (blink_node.cpp + listener_node.cpp)

---

## 5. ToA — Time of Arrival

Każdy listener mierzy absolutny czas przybycia sygnału → sfera. Trzy sfery → pozycja.

Różnica od TDoA: potrzeba absolutnej synchronizacji tagu z listenerami (nie tylko listenerów między sobą). W praktyce zastępowany przez TDoA. Używany tylko gdy tag i anchory są na tej samej szynie zegarowej.

---

## 6. PDoA — Phase Difference of Arrival

Jeden anchor, dwie anteny w odległości `d`. Różnica faz odbitego sygnału na obu antenach → kąt nadejścia.

```
      Antena A        Antena B
          |                |
          |←──── d ───────→|
           \     θ        /
            \            /
             \          /
              \        /
               \      /
                [TAG]

Δφ = 2π × d × sin(θ) / λ
θ = arcsin(Δφ × λ / (2π × d))
```

DW3000 mierzy PDoA sprzętowo w trybie `DWT_PDOA_M1` lub `M3`. Wymaga dwóch połączonych chipów lub jednego z podwójnym RX path.

| | |
|---|---|
| Liczba ramek | 2 (TWR + kąt) |
| Dokładność kąta | ±5–10° (LOS), ±20° (NLOS) |
| Wymaga synchronizacji | nie |
| Min. anchorów do pozycji 2D | 1 (kąt + odległość) |

---

## Porównanie zbiorcze

| Algorytm | Ramki | Dokładność | Sync listenerów | Skalowalność tagów | Rój bez anchorów |
|---|---|---|---|---|---|
| SS-TWR | 2 | 5–15 cm | nie | mała | tak (parami) |
| DS-TWR | 3 | 2–10 cm | nie | mała | tak (parami) |
| ADS-TWR | 3 | 2–5 cm | nie | mała | tak (parami) |
| TDoA | 1 | 10–30 cm | **tak** | **bardzo duża** | tak (ze synchem) |
| PDoA | 2 | 10 cm + kąt | nie | mała | ograniczone |

---

## Rekomendacja dla roju dronów

**Parami (2 pojazdy):** DS-TWR — dokładne, proste, nie wymaga synchronizacji.

**Rój N pojazdów (wzajemna lokalizacja):**
- TDMA + DS-TWR parami → macierz odległości → MDS dla pozycji względnych
- Patrz: `docs/swarm_sync.md` — szczegółowy opis protokołów bez stałych anchorów

**Rój z bazą (znane pozycje listenerów):** TDoA — skalowalność, jeden blink obsługuje N pojazdów jednocześnie.

---

## Wpływ geometrii na dokładność (DOP)

Niezależnie od algorytmu, dokładność pozycji zależy od rozmieszczenia anchorów/listenerów — tzw. **DOP (Dilution of Precision)**:

```
                Dobra geometria          Zła geometria
Anchory:     ●           ●           ●    ●    ●
             (w narożach)            (w jednej linii)

GDOP:        ~1.0                    ~10+
Błąd pos.:   = błąd_pomiaru × GDOP
```

Reguła: anchory/listenery rozmieszczone równomiernie wokół obszaru roboczego minimalizują DOP. Unikaj układania ich w linii.
