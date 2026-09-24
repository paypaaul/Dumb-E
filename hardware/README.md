# Hardware

Parti meccaniche (stampate in 3D), elettronica e distinta base.

- `pcb/` — PCB `test-dumbev2` (EasyEDA Pro): progetto, netlist, schematico PDF, Gerber, DXF.
  **Leggi [pcb/REVIEW.md](pcb/REVIEW.md)** prima di alimentarla: contiene pinout, problemi e interventi.
- `cad/` — modelli e STL (da aggiungere).
- [BOM.md](BOM.md) — distinta base (bozza).

## Giunti

| Giunto | Motore | Driver | Riduttore | Rapporto | Microstep |
|---|---|---|---|---|---|
| J1 base | NEMA17 | TMC2209 (U1) | cicloidale | da misurare | da jumper |
| J2 spalla | NEMA17 | TMC2209 (U2) | cicloidale | da misurare | da jumper |
| J3 gomito | NEMA17 | TMC2209 (U3) | cicloidale | da misurare | da jumper |
| J4 pitch polso | NEMA17 | TMC2209 (U4) | cicloidale | da misurare | da jumper |
| J5 roll polso | NEMA17 | TMC2209 (U5) | cicloidale | da misurare | da jumper |
| Gripper | servo (H7) | — | — | — | — |

I valori vanno riportati in `firmware/components/robot/robot_config.c`.
