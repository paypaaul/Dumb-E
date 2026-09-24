# Bring-up: cablaggio e prove al banco

> **Mappa pin provvisoria.** È pensata per un ESP32 DevKit WROOM-32 e verrà sostituita dal pinout del PCB
> del progetto dopo la sua revisione. L'unico file da cambiare è `firmware/components/board/board.c`
> (e `board.h`).

## Mappa pin

| Segnale | GPIO | Note |
|---|---|---|
| J1 base STEP / DIR | 25 / 33 | tutti gli STEP < 32 (una sola scrittura di registro) |
| J2 spalla STEP / DIR | 26 / 32 | |
| J3 gomito STEP / DIR | 27 / 14 | 14 è un pin JTAG: niente JTAG esterno |
| J4 pitch polso STEP / DIR | 19 / 18 | |
| J5 roll polso STEP / DIR | 23 / 4 | |
| EN driver (comune, attivo basso) | 13 | **pull-up esterno 10 kΩ verso 3V3** |
| Servo gripper | 22 | |
| LED di stato | 2 | LED della scheda |
| E-stop | 34 | solo ingresso; pulsante NC verso GND + pull-up esterno 10 kΩ; da abilitare in menuconfig |
| Riserve | 35, 36, 39 (finecorsa), 16/17 (UART TMC2209), 21 (I2C) | |

Pin da evitare sull'ESP32 classico: 0, 2, 5, 12, 15 (strapping; il 12 alto al boot porta la flash a 1,8 V e
la scheda non parte), 6–11 (flash), 1/3 (console). 34–39 sono solo ingressi e non hanno pull-up interni.

## Note di cablaggio

- **EN con pull-up**: così i driver restano disabilitati durante boot, reset e flash.
- **TMC2209 standalone**: MS1/MS2 fissano i microstep (devono coincidere con `microsteps` in config), la
  corrente si regola col trimmer VREF. Valuta SpreadCycle (pin SPREAD alto) sui giunti caricati dalla
  gravità.
- Alimentazione motori a 24 V con condensatore elettrolitico vicino a ogni driver; GND in comune con l'ESP32.
- **Mai** collegare o scollegare un motore con il driver alimentato.
- Il servo del gripper va alimentato separatamente dalla logica (GND in comune).
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

1. Flash e monitor: `idf.py -p <porta> flash monitor`. Al boot: `status` → `state=DISABLED ref=0`.
2. `config`: verifica microstep e rapporti.
3. `enable`, poi `zero`: `status` → `state=READY`, passi della posa di parcheggio.
4. Un giunto per volta: `jog 1 10 -v 20`, `jog 1 -10 -v 20`. Controlla il verso (altrimenti `invert`).
5. Analizzatore logico su STEP/DIR di un asse:
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
