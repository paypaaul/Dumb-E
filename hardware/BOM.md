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
| 1 | Alimentatore switching 24 V 15 A 360 W (ingresso 110/220 V selezionabile) | https://amzn.eu/d/03CJn3oo | **metti il selettore su 220 V prima di collegarlo**; protezioni: sovratensione, sovraccarico, corto |
| 1 | Step-down DC-DC 8–40 V → 5 V 5 A (25 W), sincrono | https://amzn.eu/d/0fcRmi94 | ingresso rosso/nero, uscita giallo/nero → CN5; non usarlo a lungo vicino ai 5 A |
| 1 | Presa IEC320 C14 con interruttore luminoso e portafusibile (6 A 250 V AC), cavi 18 AWG | https://amzn.eu/d/01dDmsnM | lato **rete 230 V**; montare il fusibile **F5A**, non il 10 A (vedi sotto) |
| TBD | Ventole 5 V per i TMC2209 | TBD | dimensione e corrente **TBD** |
| 1 | Portafusibile + fusibile 5 A sulla linea 24 V motori | da scegliere | consigliato (vedi stima sotto) |
| 1 | Elettrolitico 470–1000 µF 10 V vicino ai servo | da scegliere | consigliato |

### Bilancio dei consumi

| Carico | Stima |
|---|---|
| 5 × NEMA17 (corrente assorbita dai 24 V, non di fase) | ~0,3–0,7 A ciascuno → ~3 A di picco |
| 5 V: ESP32 ~0,25 A + servo gripper (stallo fino a ~2,5 A) + ventole | ~3 A di picco → 60 % dello step-down (5 A) ✓ |
| **Totale lato 24 V** | **~4 A di picco su 15 A disponibili** ✓ |

L'alimentatore è ampiamente sovradimensionato: va bene, ma proprio per questo può erogare 15 A dentro un guasto
prima che la sua protezione intervenga. Sul PCB la pista che collega i due morsetti motori è da 0,25 mm
(vedi `pcb/REVIEW.md`), quindi **serve un fusibile da 5 A** (ritardato) sul positivo 24 V verso i driver.
La presa C14 con fusibile è sul lato rete (230 V): protegge l'alimentatore, non la linea dei motori.

### Lato rete (230 V)

- A pieno carico l'alimentatore assorbe ~360 W / 0,85 / 230 V ≈ 1,9 A dalla rete; con il robot ~0,5 A.
- Usa il fusibile **F5A** in dotazione: l'interruttore è dato per **6 A a 250 V**, quindi il fusibile da 10 A
  supererebbe la sua portata.
- Collega la **terra (PE)** della presa al morsetto di terra dell'alimentatore e a eventuali parti metalliche
  del contenitore; L e N ai morsetti L/N dell'alimentatore.
- Morsetti a 230 V dell'alimentatore protetti (copertura) e cavi fissati: sono tensioni pericolose.

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
