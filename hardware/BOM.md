# Distinta base (BOM)

Stato: bozza. **TBD** = dato da completare. Le quantità del PCB vengono dalla netlist (`pcb/`).

## Elettronica di controllo (PCB `test-dumbev2`)

| Rif. | Q.tà | Componente | Note |
|---|---|---|---|
| U8 | 1 | DOIT ESP32 DevKit V1, 30 pin (WROOM-32, CP2102) | su zoccolo |
| U1…U5 | 5 | TMC2209 SilentStepStick | U6 non utilizzabile (vedi `pcb/REVIEW.md`), non serve montarlo |
| C1…C6 | 6 | Elettrolitico 470 µF 50 V (passo 5 mm, Ø10) | anche per U6 se lo monti |
| U7, U9 | 2 | Morsetto a vite KF128L-5.0-2P | alimentare **entrambi** a 24 V |
| H1…H6 | 6 | Connettore A2502WV-04P2 (motori) | + 6 connettori volanti A2502-04Y e contatti |
| CN1…CN5 | 5 | Connettore A2502-WV02 (+5V/GND) | CN5 = ingresso 5 V |
| H7, H8 | 2 | Header 3 pin 2,54 mm (servo) | |
| H9, H10 | 2 | Header 3 pin 2,54 mm + jumper | MS2 / MS1 |
| H11, H12 | 2 | Header 4 pin 2,54 mm | espansioni |
| H13…H16 | 4 | Header 3 pin 2,54 mm | uscite VCC / +5V / GND / +3V3 |
| — | 1 | Resistenza 10 kΩ (0603/0805 o THT) | **rework** pull-up EN sulle piazzole di Q3 |
| LED1…7, R4…R10, Q3 | — | LED, 150 Ω, 1 kΩ, AO3401A | **non montati** (opzionali) |

## Alimentazione

| Q.tà | Componente | Link | Note |
|---|---|---|---|
| 1 | Alimentatore 24 V | https://amzn.eu/d/03CJn3oo | potenza **TBD**; consigliati ≥ 150 W |
| 1 | Step-down DC-DC 24 → 5 V | https://amzn.eu/d/0fcRmi94 | corrente **TBD**; servono ≥ 3 A (servo + ESP32 + ventole) |
| 1 | Presa/interruttore con fusibile | https://amzn.eu/d/01dDmsnM | lato rete o 24 V? valore fusibile **TBD** |
| TBD | Ventole 5 V per i TMC2209 | TBD | dimensione e corrente **TBD** |
| 1 | Portafusibile + fusibile 5 A sulla linea 24 V motori | da scegliere | consigliato (vedi stima sotto) |
| 1 | Elettrolitico 470–1000 µF 10 V vicino ai servo | da scegliere | consigliato |

### Stima dei consumi (da verificare con i dati reali)

| Carico | Stima |
|---|---|
| 5 × NEMA17 (corrente dall'alimentazione a 24 V, non di fase) | ~0,3–0,7 A ciascuno → ~3 A di picco |
| 5 V: ESP32 ~0,25 A + servo gripper (stallo fino a ~2,5 A) + ventole | ~3 A → ~0,8 A a 24 V |
| **Totale lato 24 V** | **~4 A → alimentatore ≥ 100 W, consigliato 150 W** |

## Attuatori

| Q.tà | Componente | Note |
|---|---|---|
| 5 | NEMA17 (1,8°) | nel CAD: 4 standard + 1 pancake (J4); modello e corrente **TBD** |
| 1 | Servo gripper (gripper a 3 dita con ingranaggio) | modello **TBD** |

## Meccanica

| Q.tà | Componente | Note |
|---|---|---|
| 5 | Riduttore cicloidale stampato 20:1 | corona a 20 perni Ø3 mm, 2 dischi sfasati (dal CAD) |
| TBD | Cuscinetti, rulli/perni, viteria | **TBD** dal CAD |
| TBD | Filamento | materiale **TBD** (PETG/ASA consigliati vicino ai motori) |
