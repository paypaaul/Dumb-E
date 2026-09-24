# Revisione PCB `test-dumbev2` (V1.0, schematico aggiornato 2026-01-11)

Revisione fatta su netlist (`.enet`), progetto EasyEDA Pro (`.epro`), PDF dello schematico e foto della DevKit
montata. Il firmware usa il pinout qui sotto (`firmware/components/board/board.c`).

## Cosa c'è sulla scheda

- **U8**: zoccolo per **DOIT ESP32 DevKit V1 a 30 pin** (WROOM-32, CP2102). Nel progetto il simbolo è quello
  dell'ESP32-H2 DevKit, ma footprint e collegamenti corrispondono alla DOIT (verificato: VIN → +5V, 3V3 → +3V3,
  GND → GND).
- **U1…U6**: 6 moduli TMC2209 SilentStepStick, VIO a 3,3 V, VM da VCC, 470 µF / 50 V ciascuno.
- **H1…H6**: connettori motore a 4 poli (1 M2B, 2 M2A, 3 M1A, 4 M1B).
- **H9 / H10**: jumper MS2 / MS1 **comuni a tutti i driver** (GND o 3V3).
- **U7 / U9**: due morsetti di alimentazione motori (VCC/GND).
- **CN5** (e CN1…CN4, H14): ingresso/uscite +5V. **Non c'è un regolatore 5 V**: va fornito da fuori.
- **H7 / H8**: connettori servo (GND, +5V, segnale).
- **H11 / H12**: espansioni a 4 pin; **H13…H16**: uscite VCC, +5V, GND, +3V3.
- **Q3** (AO3401) + LED1…6 (R4…R9): LED accesi quando i driver sono abilitati; **LED7** (R10): presenza +5V.
  **Sulla scheda attuale questi LED non sono montati**, e la scheda è pensata per funzionare anche senza (vedi
  sotto).

### LED non montati

LED1…7, le loro resistenze e Q3 sono solo indicatori: stanno sulla linea EN (gate di Q3) e sulle alimentazioni
+3V3/+5V, e nessun altro circuito dipende da loro. Verificato sulla netlist: senza di loro la linea EN collega
solo GPIO25 e gli EN dei sei driver, e +5V/+3V3 restano invariati. Il firmware non li usa (il LED di stato è quello
della DevKit su GPIO2). Nella prossima revisione conviene tenerli come opzionali ("DNP"), senza collegarci
funzioni essenziali: il pull-up di EN, per esempio, deve essere una resistenza a sé, non dipendere da Q3.

## Pinout (DOIT DevKit V1)

| Segnale | GPIO | Giunto nel firmware | Note |
|---|---|---|---|
| U1 STEP / DIR | 26 / 27 | J1 base | |
| U2 STEP / DIR | **12** / 13 | J2 spalla | GPIO12 è strapping (vedi problema 3) |
| U3 STEP / DIR | 32 / 33 | J3 gomito | |
| U4 STEP / DIR | 4 / 16 | J4 pitch polso | GPIO16 non esiste sulle DevKit WROVER |
| U5 STEP / DIR | 18 / 19 | J5 roll polso | |
| U6 STEP / DIR | **35 / 34** | — | **inutilizzabile**: pin solo ingresso |
| EN (tutti) + gate Q3 | 25 | | nessun pull-up (problema 2); Q3/LED non montati |
| Servo H7 / H8 | 23 / 22 | gripper su H7 | |
| H11 | 15, 2, 3 (RX0), 1 (TX0) | | 2 e 15 strapping, 1/3 console |
| H12 | 21, 5, 17, 14 | e-stop su 21 (opzionale) | 5 strapping |
| LED di stato | 2 | LED della DevKit | anche su H11.2 |

## Problemi e interventi

### Da fare prima di dare potenza ai motori

1. **Alimenta entrambi i morsetti U7 e U9.** Ciascun morsetto alimenta una fila di tre driver con piste da
   30 mil; tra i due c'è solo una pista da **10 mil lunga 54 mm**. Con un solo morsetto collegato, metà della
   corrente passa da quella pista, che può scaldarsi fino a interrompersi.
2. **Pull-up su EN.** La linea EN (GPIO25) non ha resistenze: durante accensione, reset e flash è flottante,
   quindi i driver sono in uno stato indefinito e i motori possono attivarsi.
   **Intervento:** resistenza da **10 kΩ tra le piazzole di gate e source di Q3** (la source è a +3V3, il gate è
   la linea EN). Q3 non è montato, quindi le piazzole SOT-23 sono libere: una resistenza 0603/0805 ci sta tra i
   due pad, oppure un pezzo di resistenza THT. In alternativa: 10 kΩ tra un pin +3V3 di H16 e la linea EN su un
   pin EN di un driver (pin 16 del modulo).
3. **GPIO12 (STEP di U2).** È il pin che all'avvio sceglie la tensione della flash: se è alto la scheda non parte.
   Normalmente il pull-down interno lo tiene basso. Verifica che la scheda si avvii e si flashi con i driver montati
   e alimentati. Se non parte, la soluzione standard sui moduli WROOM-32 (flash a 3,3 V) è
   `espefuse.py --port <porta> set_flash_voltage 3.3V`: **è irreversibile**, fallo solo se serve e solo su WROOM-32.

### Alimentazione

4. Nessun fusibile, protezione da inversione di polarità o TVS sull'ingresso motori: aggiungili almeno sul cavo
   (fusibile sull'alimentatore; per la prossima revisione MOSFET di protezione e TVS, es. SMBJ30A per 24 V).
5. Il +5V arriva da fuori (CN5) e alimenta sia l'ESP32 (VIN) sia i servo: un servo sotto sforzo può far scendere
   la tensione e resettare l'ESP32. Usa un alimentatore 5 V da almeno 3 A e un condensatore (470–1000 µF) vicino
   ai connettori servo.
6. Con USB e 5 V esterno collegati insieme, verifica che la tua DevKit abbia il diodo tra USB e VIN, per non
   alimentare il PC dal 5 V esterno.

### Minori

7. U6 non è utilizzabile (pin solo ingresso). Per 5 giunti non serve.
8. Non collegare nulla a TX0/RX0 su H11: è la seriale del protocollo e del flash.
9. PDN_UART, DIAG, INDEX e SPREAD non sono collegati e MS1/MS2 sono comuni: i driver funzionano in standalone
   (microstep dai jumper, corrente dal trimmer). Una UART condivisa in futuro richiederebbe indirizzi diversi.
10. Massa: pour su entrambi i lati, bene. 470 µF per driver: bene.

## Per la prossima revisione del PCB

- Simbolo corretto (ESP32 DevKit V1 30 pin) al posto di quello dell'H2.
- U6 su due uscite libere (per esempio 14 e 21, oggi su H12) oppure eliminarlo.
- STEP di U2 lontano da GPIO12.
- Pull-up 10 kΩ su EN come componente dedicato (non affidato a Q3).
- LED e Q3 marcati come opzionali (DNP): la scheda deve funzionare identica con o senza.
- VCC con piste larghe (≥ 80 mil) o pour, un solo ingresso motori con fusibile, protezione da inversione e TVS.
- Regolatore 5 V dedicato ai servo, separato da quello dell'ESP32.
- PDN_UART collegata a una UART dell'ESP32 (con resistenza da 1 kΩ) e MS1/MS2 separati per gli indirizzi.
- Ingressi per finecorsa ed e-stop con connettori dedicati.
