# Roadmap

| Fase | Contenuto | Criterio di uscita | Stato |
|---|---|---|---|
| **0 — Igiene e tooling** | monorepo, ESP-IDF v6.0.3 fissato, `sdkconfig.defaults`, CI, README e docs | CI verde | fatto |
| **1 — Fondamenta del moto** | stepgen, pianificazione MOVEJ sincronizzata, cinematica 5-DOF + test, stati, gripper, protocollo v0 | test verdi, build ok, prove al banco | software fatto; prove al banco da fare |
| 1b — PCB | revisione dello schematico, pinout definitivo in `board.c` | pinout verificato | in attesa dei file |
| 2 — Riferimento e calibrazione | parametri e calibrazione in NVS, procedura di zero guidata, jog migliorato, **CLI Python** per script e sequenze | zero ripetibile entro una tolleranza misurata | |
| 2b — Upgrade hardware consigliati | finecorsa o sensori hall + homing; linea PDN_UART dei TMC2209 → corrente, corrente di mantenimento, SpreadCycle/StealthChop, diagnostica e rilevamento del reset del driver | homing ripetibile, config dei driver verificata | |
| 3 — Cinematica completa | MOVEL cartesiano (IK a 1 kHz), validazione del percorso prima del moto, profili S-curve, jog cartesiano, TCP configurabile | movimenti fluidi, errore misurato | |
| 4 — Rete e tool | specifica protocollo v1, trasporto TCP/WebSocket, telemetria, teach & replay dal PC, eventuale bridge ROS 2 lato PC | controllo remoto end-to-end | |
| 5 — Affidabilità | encoder sui giunti (passi persi), e-stop hardware, monitor della tensione motori, watchdog, OTA | test di durata | |

## Note hardware

- Senza finecorsa la ripetibilità dipende dalla procedura di zero manuale: la Fase 2b è fortemente
  consigliata.
- Homing sensorless (StallGuard) sconsigliato attraverso un cicloidale: la coppia in uscita, moltiplicata
  dal rapporto, può rompere le parti stampate prima che lo stallo venga rilevato. Inoltre richiede la UART.
- I registri del TMC2209 si perdono se cade la tensione motori: con la UART il firmware dovrà rilevarlo
  (`GSTAT.reset`) e riconfigurare i driver, considerando perso il riferimento.
- Con 6 assi, finecorsa ed encoder il numero di pin dell'ESP32 classico è al limite: valutare ESP32-S3 o un
  expander nella prossima revisione del PCB.
