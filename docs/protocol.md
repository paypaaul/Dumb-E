# Protocollo seriale v0

Interfaccia a riga di testo su UART0 (115200 baud, quella di `idf.py monitor`). Pensata sia per l'uso a
mano sia per un client (es. script Python con pyserial).

## Formato

- Una richiesta per riga: `comando arg1 arg2 ... [-opzione valore]`.
- Ogni comando termina con **una sola riga di esito**, scritta in un colpo solo:
  - `ok [chiave=valore ...]`
  - `err <codice> <messaggio>`
- Le righe che iniziano con `#` sono dettagli informativi che precedono la riga di esito.
- Tutto il resto (log `I (...)`, `W (...)`, prompt `dumbe>`) va ignorato dal client.
- Angoli in gradi, distanze in mm, velocità e accelerazioni in percentuale dei limiti del giunto.

## Comandi

| Comando | Descrizione |
|---|---|
| `status` | stato, giunti, passi, TCP, gripper, statistiche dello stepgen |
| `config` | configurazione meccanica (righe `#`) |
| `version` | versione firmware, ESP-IDF, protocollo |
| `enable` / `disable` | alimenta / spegne i driver (`disable` perde il riferimento: il braccio può cadere) |
| `zero [q1..q5]` | dichiara gli angoli attuali (default: posa di parcheggio) |
| `movej q1 q2 q3 q4 q5 [-v %] [-a %]` | movimento sincronizzato ai giunti |
| `movep x y z pitch roll [-e up\|down] [-v %] [-a %]` | IK + movimento ai giunti verso una posa del TCP |
| `jog giunto(1-5) delta [-v %]` | movimento relativo di un giunto |
| `park [-v %]` | torna alla posa di parcheggio |
| `stop` | arresto controllato, scarta i comandi in coda |
| `estop` | arresto immediato, bloccato fino a `reset` |
| `reset` | sblocca l'e-stop (il riferimento resta: verifica la posizione) |
| `fk q1..q5` | cinematica diretta (non muove) |
| `ik x y z pitch roll [-e up\|down]` | cinematica inversa (non muove) |
| `grip open\|close\|off\|<0-100>` | gripper; `off` smette di pilotare il servo |
| `wifi [ssid [password]]` | stato WiFi, oppure salva le credenziali (solo da fermo) |

Default: `-v 50 -a 50`, `-e up`. I movimenti vengono accodati (fino a 8) ed eseguiti in ordine.

## Risposta di `status`

```
ok state=READY ref=1 moving=0 estop_in=0 queue=0 q=0.000,90.000,-90.000,0.000,0.000
   steps=0,32000,-32000,0,0 tcp=210.00,0.00,250.00,0.00,0.00 grip=off ticks=123456
   isr_us=1.20/3.40 segments=27473 underruns=0 rejected=0 stalls=0
```
(su una sola riga)

| Chiave | Significato |
|---|---|
| `state` | `DISABLED`, `ENABLED` (non riferito), `READY`, `ESTOP` |
| `ref` | 1 se riferito |
| `moving` | 1 se in moto o con comandi in coda |
| `q` | angoli comandati [°] |
| `steps` | posizioni emesse dallo stepgen |
| `tcp` | x,y,z [mm], pitch,roll [°]; `na` se non riferito |
| `isr_us` | durata media/massima dell'ISR [µs] |
| `underruns` | coda vuota a metà movimento (deve restare 0) |
| `rejected` | segmenti scartati per troppi passi (deve restare 0) |
| `stalls` | allarmi del timer persi e riarmati (deve restare 0 sull'hardware) |

## Codici di errore

| Codice | Significato |
|---|---|
| 1 | argomento non valido |
| 2 | non permesso in questo stato (serve `enable`) |
| 3 | non riferito (serve `zero`) |
| 4 | occupato (in movimento) |
| 5 | limite di un giunto |
| 6 | posa irraggiungibile |
| 7 | posa singolare (sull'asse della base) |
| 8 | e-stop attivo |
| 9 | coda comandi piena |
| 10 | errore interno |
| 100 | sintassi (il messaggio riporta l'uso corretto) |
| 101 | errore di rete |
