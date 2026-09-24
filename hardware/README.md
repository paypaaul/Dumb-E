# Hardware

Parti meccaniche (stampate in 3D), elettronica e distinta base.

- `pcb/` — PCB `test-dumbev2` (EasyEDA Pro): progetto, netlist, schematico PDF, Gerber, DXF.
  **Leggi [pcb/REVIEW.md](pcb/REVIEW.md)** prima di alimentarla: contiene pinout, problemi e interventi.
- `cad/robot.step` — modello completo del braccio (fonte della geometria in `docs/kinematics.md`).
- [BOM.md](BOM.md) — distinta base (bozza).

## Giunti

| Giunto | Motore | Driver | Riduttore | Rapporto | Microstep |
|---|---|---|---|---|---|
| J1 base | NEMA17 | TMC2209 (U1) | cicloidale | 20:1 | da jumper |
| J2 spalla | NEMA17 | TMC2209 (U2) | cicloidale | 20:1 | da jumper |
| J3 gomito | NEMA17 | TMC2209 (U3) | cicloidale | 20:1 | da jumper |
| J4 roll avambraccio | NEMA17 | TMC2209 (U4) | cicloidale | 20:1 | da jumper |
| J5 pitch polso | NEMA17 | TMC2209 (U5) | cicloidale | 20:1 | da jumper |
| Gripper | servo (H7) | — | — | — | — |

I valori vanno riportati in `firmware/components/robot/robot_config.c`.
