# Dumb-E

Braccio robotico a 5 gradi di libertà + gripper, stampato in 3D, con riduttori cicloidali su ogni giunto.

- **Controller**: ESP32 (DevKit WROOM-32), firmware in C su ESP-IDF v6.0.3
- **Attuatori**: NEMA17 + TMC2209 (standalone, STEP/DIR), servo per il gripper
- **Giunti**: J1 base (yaw), J2 spalla, J3 gomito, J4 pitch polso, J5 roll polso

> Stato: fondamenta del firmware (Fase 1). Geometria, rapporti di riduzione e limiti in
> `firmware/components/robot/robot_config.c` sono **segnaposto** da misurare; la mappa pin in
> `firmware/components/board/board.c` è provvisoria finché non viene revisionato il PCB.

## Struttura

```
firmware/          progetto ESP-IDF
  main/            bring-up (app_main)
  components/
    kinematics/    FK/IK 5-DOF, mappa giunto↔step (C puro, testato su PC)
    motion/        profili di moto + task di pianificazione a 1 kHz (core 1)
    stepgen/       generatore di impulsi STEP/DIR (ISR 40 kHz, core 1)
    robot/         macchina a stati, sicurezza, configurazione meccanica
    board/         mappa pin
    gripper/       servo su LEDC
    comms/         protocollo seriale v0 (esp_console)
    net/           WiFi (credenziali in NVS)
  test/host/       unit test su PC (CMake + Unity)
  test/qemu/       smoke test end-to-end in QEMU
hardware/          PCB, CAD/STL, BOM
tools/             strumenti lato PC
docs/              architettura, protocollo, cinematica, bring-up, roadmap, code review
```

## Compilare e flashare

Serve ESP-IDF **v6.0.3** (oppure il devcontainer in `.devcontainer/`).

```sh
cd firmware
idf.py set-target esp32      # solo la prima volta
idf.py build
idf.py -p /dev/tty.usbserial-0001 flash monitor
```

In VS Code apri la cartella `firmware/` per usare i comandi dell'estensione ESP-IDF; clangd trova
`firmware/build/compile_commands.json` tramite `.clangd`.

## Test

```sh
# unit test su PC (niente ESP-IDF richiesto)
cmake -S firmware/test/host -B build-host && cmake --build build-host && ctest --test-dir build-host

# smoke test in QEMU (vedi docs/bringup.md)
cd firmware
idf.py -B build-qemu -DSDKCONFIG=build-qemu/sdkconfig \
    -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;test/qemu/sdkconfig.qemu" build
python test/qemu/smoke_test.py build-qemu
```

La CI (`.github/workflows/ci.yml`) esegue i test su PC e la build del firmware a ogni push.

## Uso rapido

Dal monitor seriale (`idf.py monitor`, prompt `dumbe>`):

```
enable                      # driver alimentati, braccio in coppia
zero                        # il braccio è nella posa di parcheggio: riferimento impostato
movej 10 80 -60 20 45       # giunti in gradi
movep 200 0 150 -90 0       # TCP in mm, pitch e roll in gradi
grip close
park
status
```

Comandi e formato delle risposte: [docs/protocol.md](docs/protocol.md).

## Documentazione

- [Architettura](docs/architecture.md)
- [Protocollo seriale v0](docs/protocol.md)
- [Cinematica e parametri da misurare](docs/kinematics.md)
- [Bring-up: cablaggio e prove al banco](docs/bringup.md)
- [Roadmap](docs/roadmap.md)
- [Code review del prototipo iniziale](docs/code-review.md)
