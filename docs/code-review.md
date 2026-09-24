# Code review del prototipo iniziale (settembre 2026)

Revisione del commit iniziale (`e1424ef`) prima della ristrutturazione. Tutti i punti sono stati risolti
dalla riscrittura della Fase 1; il documento resta come riferimento.

## Cosa faceva

- `main.c`: init NVS → WiFi con credenziali nel codice → `ik_init()` (mai usata) → 2 motori su 4
  (32000 passi = 90°) → driver abilitati → **al boot** movimento a 90°/45° → loop vuoto.
- `stepper_driver`: ISR GPTimer a 20 kHz con accumulatore di fase `float`; task a 100 Hz sul core 1 con
  rampa trapezoidale per asse; "interpolazione lineare" = v e a scalate per il rapporto delle distanze.
- `robot_kinematics`: IK geometrica 4-DOF, output in gradi.
- `wifi_connection`: STA con riconnessione.

## Problemi critici

1. **Float nell'ISR** (`stepper_driver.c:37-41`): in IDF 6 la FPU negli ISR non è supportata di default →
   panic "Coprocessor exception" o corruzione dei registri FPU appena un motore parte.
2. **Velocità fuori scala e accumulatore mai azzerato**: 400 °/s richiesti ≈ 142.000 passi/s contro un
   massimo di 20.000; l'accumulatore cresceva senza limite e il movimento successivo partiva a piena velocità
   senza rampa.
3. **Race tra i core**: `motors[]` condiviso da ISR (core 0), planner (core 1) e API senza sincronizzazione.
4. **Cambio di target durante il moto**: inversione istantanea a piena velocità, oscillazione ±1 passo,
   divisione per zero con accelerazione nulla.
5. **Nessun riferimento né stato sicuro al boot**: movimento assoluto all'accensione.
6. **Decelerazione**: velocità che poteva diventare negativa; "crawl" a 10 passi/s non scalato.

## Problemi alti

7. "Linear interpolated" non era lineare (spazio giunti) e perdeva la sincronizzazione con rapporti diversi;
   lunghezza dell'array implicita (lettura fuori bounds con più motori).
8. IK: la soluzione "gomito in su" era gomito in giù; mancava la lunghezza polso→utensile; unità miste
   (`phi` in radianti, output in gradi); double su un chip senza FPU double; niente FK, limiti, singolarità;
   era 4-DOF.
9. Impulsi: jitter fino a 50 µs, busy-wait per l'impulso, `gpio_set_level` (in flash) chiamata dall'ISR,
   ISR sullo stesso core del WiFi.
10. GPIO: configurazione incompleta; **GPIO12 è strapping** (può impedire il boot); 12/13 sono pin JTAG.
11. Planner con `vTaskDelay(10 ms)` = 1 tick a 100 Hz, non periodico, `dt` assunto costante.

## Problemi medi

12. `stepper_is_moving()` dichiarata e mai definita; errori (`esp_err_t`) ignorati ovunque; header pubblici
    con dipendenze inutili; troncamento invece di arrotondamento.
13. WiFi: credenziali nel sorgente; `strcpy` senza limiti (overflow) e senza `<string.h>`; nessuno stato di
    connessione; retry senza backoff.
14. CMake: kinematics dipendeva dal driver dei motori; `wifi_connection` dipendeva da se stesso; componente
    `driver` deprecato in IDF 6.
15. Tooling: CMake 3.5, versione nel nome del progetto, README del template, `.DS_Store` e impostazioni
    personali committate, devcontainer non versionato, nessun `sdkconfig.defaults`, test o CI.

## Come sono stati risolti

| Problema | Soluzione |
|---|---|
| 1, 9 | ISR solo interi, in IRAM, cache-safe, scritture dirette sui registri, sul core 1 |
| 2, 6, 7, 11 | profili in forma chiusa su un parametro di percorso comune, segmenti da 1 ms, limiti validati contro la capacità dello stepgen |
| 3 | coda di segmenti lock-free (SPSC) tra task e ISR; stato condiviso protetto |
| 4 | i comandi sono accodati; `stop` decelera lungo il percorso; `estop` interrompe e riallinea |
| 5 | macchina a stati: al boot driver disabilitati e non riferito |
| 8 | cinematica 5-DOF in forma chiusa con test su PC |
| 10 | mappa pin unica senza pin di strapping per STEP/DIR/EN |
| 12–15 | componenti ristrutturati, errori propagati, WiFi con credenziali in NVS, tooling e CI |
