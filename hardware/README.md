# Hardware

Parti meccaniche (stampate in 3D), elettronica e distinta base.

- `pcb/` — sorgenti del PCB (KiCad: `.kicad_pro`, `.kicad_sch`, `.kicad_pcb`; oppure export EasyEDA /
  netlist) e PDF dello schematico. Dopo la revisione, il pinout diventa il riferimento del firmware
  (`firmware/components/board/board.c`).
- `cad/` — modelli e STL (da aggiungere).
- BOM — da aggiungere.

## Giunti

| Giunto | Motore | Driver | Riduttore | Rapporto | Microstep |
|---|---|---|---|---|---|
| J1 base | NEMA17 | TMC2209 | cicloidale | da misurare | da jumper |
| J2 spalla | NEMA17 | TMC2209 | cicloidale | da misurare | da jumper |
| J3 gomito | NEMA17 | TMC2209 | cicloidale | da misurare | da jumper |
| J4 pitch polso | NEMA17 | TMC2209 | cicloidale | da misurare | da jumper |
| J5 roll polso | NEMA17 | TMC2209 | cicloidale | da misurare | da jumper |
| Gripper | servo | — | — | — | — |

I valori vanno riportati in `firmware/components/robot/robot_config.c`.
