# Bring-up: cablaggio e prove al banco

Scheda: PCB `test-dumbev2` con DOIT ESP32 DevKit V1 (30 pin). Revisione completa, problemi e interventi
consigliati: [hardware/pcb/REVIEW.md](../hardware/pcb/REVIEW.md). Il pinout sta solo in
`firmware/components/board/board.c`.

## Mappa pin

| Segnale | GPIO | Note |
|---|---|---|
| J1 base (U1) STEP / DIR | 26 / 27 | motore su H1 |
| J2 spalla (U2) STEP / DIR | 12 / 13 | GPIO12 è strapping: vedi REVIEW.md |
| J3 gomito (U3) STEP / DIR | 32 / 33 | motore su H3 |
| J4 roll avambraccio (U4) STEP / DIR | 4 / 16 | motore su H4 |
| J5 pitch polso (U5) STEP / DIR | 18 / 19 | motore su H5 |
| U6 | 35 / 34 | inutilizzabile (pin solo ingresso) |
| EN driver (comune, attivo basso) | 25 | serve un pull-up 10 kΩ (sulle piazzole libere di Q3, vedi REVIEW.md) |
| Servo gripper | 23 | connettore H7 (H8 = GPIO22, di riserva) |
| LED di stato | 2 | LED della DevKit |
| E-stop (opzionale) | 21 | H12 pin 1; pulsante NC verso GND; da abilitare in menuconfig |

Se colleghi i motori a zoccoli diversi, cambia l'assegnazione in `board.c`.

## Note di cablaggio

- **Alimenta entrambi i morsetti motori U7 e U9** (vedi REVIEW.md).
- **EN con pull-up**: così i driver restano disabilitati durante boot, reset e flash.
- **TMC2209 standalone**: i jumper H10 (MS1) e H9 (MS2) fissano i microstep per tutti i driver e devono
  coincidere con `microsteps` in config; la corrente si regola col trimmer VREF.
- Alimentazione motori a 24 V; il 5 V (CN5) va fornito da un alimentatore esterno da almeno 3 A.
- **Mai** collegare o scollegare un motore con il driver alimentato.
- Non collegare nulla a TX0/RX0 su H11 (seriale del protocollo).
- I cicloidali sono reversibili: con i driver disabilitati il braccio può cadere. La posa di parcheggio deve
  essere stabile.

## LED di stato

| LED | Stato |
|---|---|
| lampo breve ogni 2 s | DISABLED |
| lampeggio lento | ENABLED (non riferito) |
| acceso fisso | READY |
| lampeggio medio | in movimento |
| lampeggio veloce | ESTOP |

## Prove al banco

Fai le prime prove **con i motori staccati dalla meccanica** (o con il braccio smontato).

1. Prima del primo avvio: pull-up su EN e jumper MS1/MS2. Flash e monitor: `idf.py -p <porta> flash monitor`.
   La scheda deve avviarsi anche con i driver alimentati (GPIO12). Al boot: `status` → `state=DISABLED ref=0`.
   I LED della scheda (LED1…7) non sono montati: per verificare EN misura con il tester la tensione sul
   pin EN di un driver (≈ 3,3 V = disabilitato, 0 V dopo `enable`).
2. `config`: verifica microstep e rapporti.
3. `enable`, poi `zero`: `status` → `state=READY`, passi della posa di parcheggio.
4. Un giunto per volta: `jog 1 10 -v 20`, `jog 1 -10 -v 20`. Controlla il verso (altrimenti `invert`).
5. Analizzatore logico su STEP/DIR di un asse (anche J2 su GPIO12 e J3 su GPIO32):
   - impulso STEP ≥ 1 µs;
   - DIR stabile almeno un tick (25 µs) prima del primo STEP dopo un cambio;
   - frequenza massima coerente con la velocità richiesta.
6. Carico ISR: `status` → `isr_us=media/max`; la media deve restare ben sotto i 25 µs del tick (obiettivo
   < 30 %). Per una misura precisa imposta `DUMBE_STEPGEN_DEBUG_GPIO` in menuconfig e misura il pin.
7. Ripetibilità: segna la posizione di un giunto, fai 20 andate e ritorni (`movej` tra due pose), torna
   alla posa iniziale: la tacca deve coincidere (0 passi persi). `underruns`, `rejected` e `stalls` devono
   restare 0.
8. Arresti: durante un movimento lungo prova `stop` (decelera) ed `estop` (immediato), poi `reset`.
9. Gripper: `grip open`, `grip close`, `grip 50`, `grip off`. Regola `gripper_pulse_min/max_us`.
10. Solo dopo: rimonta la meccanica e ripeti con velocità (`-v`) basse, salendo gradualmente.

## Smoke test in QEMU (senza hardware)

Verifica tutto lo stack (protocollo, stati, pianificazione, stepgen) nell'emulatore:

```sh
python $IDF_PATH/tools/idf_tools.py install qemu-xtensa     # una volta
cd firmware
idf.py -B build-qemu -DSDKCONFIG=build-qemu/sdkconfig \
    -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;test/qemu/sdkconfig.qemu" build
python test/qemu/smoke_test.py build-qemu
```

In QEMU il WiFi è disattivato (non è emulato) e il tick è abbassato a 5 kHz perché l'ISR emulata è molto
più lenta di quella reale; `stalls` > 0 è normale solo nell'emulatore.
