# Brief do prezentacji — UWB Swarm: lokalizacja względna 2D

> Wklej poniższe do Claude (tryb prezentacji/artifact) z poleceniem:
> „Zrób na podstawie tego briefu prezentację (slajdy) — czysty, techniczny styl,
> ciemny motyw, diagramy. Język: polski."

---

## Kontekst projektu
- **Temat:** System lokalizacji względnej 2D dla roju urządzeń z modułami UWB.
- **Cel:** Bez żadnej infrastruktury (bez GPS, bez stałych anchorów) odtworzyć
  wzajemne pozycje N modułów tylko z pomiarów odległości między nimi.
- **Zespół/kontekst:** AGH AIOTA, projekt UWB Swarm.
- **Sprzęt:** Seeed XIAO ESP32-C3 + moduł UWB DW3000 (chip Qorvo/Decawave), PlatformIO/Arduino.

## Co zostało zbudowane (stan obecny)
- Jeden firmware na wszystkie moduły (różni je tylko `MY_ID`).
- Pomiar odległości każdy-z-każdym metodą **SS-TWR** z korekcją dryfu zegara.
- **Pełna siatka 1-hop (full mesh)** — N(N−1)/2 par odległości.
- Moduł 0 zbiera całą macierz odległości i wysyła na PC (UART).
- **Wizualizacja w Pythonie (matplotlib)** — na żywo odtwarza pozycje 2D metodą **MDS**.
- Zweryfikowane na realnym sprzęcie: stabilne pomiary ~0.6 m między modułami.

---

## Proponowana struktura slajdów

### 1. Tytuł
„UWB Swarm — lokalizacja względna 2D bez infrastruktury"
Podtytuł: ESP32-C3 + DW3000, ranging SS-TWR + MDS.

### 2. Problem
- Klasyczny UWB: stałe anchory w znanych miejscach → tag dostaje pozycję.
- Rój: nic nie jest stałe, układ odniesienia trzeba zbudować z pomiarów między pojazdami.
- Wyzwania: O(N²) par, kolizje na wspólnym medium, brak wspólnego zegara.

### 3. Zasada działania UWB / TWR
- UWB mierzy **czas przelotu** sygnału (ToF) → odległość = ToF × c.
- DW3000: znaczniki czasu 40-bit, 1 jednostka ≈ 15.65 ps.
- **SS-TWR (Single-Sided Two-Way Ranging):** 2 ramki Poll→Response.
  - `TOF = (RTD_init − RTD_resp×(1−ε)) / 2`, gdzie ε = dryf zegara (mierzony sprzętowo).
  - Korekcja dryfu zbija błąd z ~2.4 m do ~10 cm.
- (Opcjonalnie slajd o DS-TWR — 3 ramki, błąd rzędu ε² ≈ kilka cm.)

### 4. Architektura sieci (mesh)
- **Płaski full-mesh, jeden skok** — każdy mierzy się z każdym.
- **Dostęp do medium: TDMA wolnobieżne** (bez globalnego SYNC):
  `slot = (millis() / 60ms) % N`.
  - W swoim slocie: moduł pyta wszystkich innych (SS-TWR) + rozgłasza swój wiersz.
  - Poza slotem: nasłuchuje i odpowiada na zapytania.
- **Dane:** wiersze macierzy spływają broadcastem do M0 (topologia gwiazdy nad meshem).
- Diagram: 4 węzły, strzałki pomiarów (mesh) + strzałki danych do M0 → PC.

### 5. Od odległości do pozycji — MDS
- Wejście: macierz N×N odległości euklidesowych.
- **Classical MDS:** podwójne centrowanie `B = −½·H·D²·H`, rozkład własny, 2 największe wektory własne → pozycje 2D.
- Wynik jest **względny** (bez orientacji bezwzględnej) — stąd na wizualizacji „przypinamy" węzeł 1 w lewym-dolnym rogu (wyrównanie Prokrustesa).
- Wygładzanie EMA, żeby pozycje nie skakały między klatkami.

### 6. Wizualizacja
- Python + matplotlib, odczyt UART „MATRIX,d01,...".
- Na żywo: 4 węzły z kolorami, ślady ruchu, tabela odległości.
- Zrzut ekranu / mockup wykresu.

### 7. Demo / wyniki
- Realny pomiar: M0↔M2 ≈ 0.60 m, stabilne.
- Dokładność SS-TWR z korekcją: ~10 cm.
- Częstotliwość: kilka–kilkanaście Hz (zależnie od N).

### 8. Historia debugowania (highlight — ciekawy slajd)
- **Objaw:** po podłączeniu modułu UWB serial „umierał", po odłączeniu wszystko się wysypywało.
- Fałszywe tropy (i jak je odrzucono): zasilanie/brownout, PA/LNA, USB CDC, kolejność yieldów.
- **Prawdziwa przyczyna:** busy-wait bez timeoutu (`while(!TXFRS){}`) — pierwszy TX po inicjalizacji się nie kończył → MCU wisiał w nieskończonej pętli. Po odłączeniu MISO „pływał", odczyt SPI dawał same jedynki → pętla wychodziła → wysyp.
- **Lekcja:** każde oczekiwanie na zdarzenie sprzętowe musi mieć limit czasu.
- (Dodatkowo: `dwt_forcetrxoff()` w pętli też wieszał natywny USB ESP32-C3.)

### 9. Ograniczenia i dalszy rozwój
- 1-hop (brak zasięgu poza bezpośrednim), brak twardej synchronizacji, N na sztywno.
- Kierunki: DS-TWR (większa dokładność), EKF (wygładzanie trajektorii),
  twardy TDMA/multi-hop (skala i zasięg), rozproszony solver na pokładzie.

### 10. Podsumowanie
- Działający anchor-free system lokalizacji względnej 2D na tanim sprzęcie.
- Pełny łańcuch: ranging UWB → macierz odległości → MDS → wizualizacja na żywo.

---

## Fakty techniczne (do wykorzystania na slajdach)
- Chip: DW3000, kanał 5, 6.8 Mbps, PLEN 128.
- 1 DTU ≈ 15.65 ps; UUS_TO_DWT_TIME = 65536.
- Kalibracja anteny: `ANT_DLY ≈ 16395 + 17 DTU` (≈ korekcja 16 cm).
- SS-TWR: korekcja dryfu przez `clockOffsetRatio = dwt_readclockoffset()/2^26`.
- TDMA: slot 60 ms, cykl = N × 60 ms.
- Format UART z M0: `MATRIX,d01,d02,d03,d12,d13,d23` [metry].
- MDS: min. 3 moduły do 2D (wszystkie pary muszą być zmierzone).

## Sugestie wizualne
- Ciemny motyw (granat/grafit), akcenty: czerwony/niebieski/zielony/pomarańczowy (kolory węzłów).
- Diagramy: timing SS-TWR (Poll/Response z osią czasu), mesh 4 węzłów, pipeline „UWB→macierz→MDS→wykres".
- Ikony: fala radiowa, zegar, węzły sieci.
