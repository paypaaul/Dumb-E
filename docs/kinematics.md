# Cinematica

## Modello

Braccio a 5 gradi di libertà: base (yaw), spalla, gomito, pitch del polso, roll del polso. Tutti gli assi di
pitch sono paralleli e il piano del braccio contiene l'asse della base (**nessun offset laterale**: da
verificare sul CAD).

```
            d5 (polso → TCP)
   a3        ●──────▶ TCP
 ●──────────● J4 (pitch)  J5 (roll attorno all'asse di approccio)
 │ J3 (gomito)
 │ a2
 ● J2 (spalla)
 │ a1 (offset orizzontale, spesso 0)
 │ d1
═╧═ J1 (base)
```

| Parametro | Significato |
|---|---|
| `d1` | altezza dal piano di base all'asse della spalla |
| `a1` | distanza orizzontale dall'asse della base all'asse della spalla |
| `a2` | asse spalla → asse gomito |
| `a3` | asse gomito → asse di pitch del polso |
| `d5` | asse di pitch del polso → punto utensile (TCP), lungo l'asse di approccio |

## Convenzioni

- Terna di base: z verso l'alto, x in avanti con J1 = 0; J1 positivo antiorario visto dall'alto.
- **q2** è l'angolo del braccio rispetto all'orizzontale (positivo = verso l'alto).
- **q3** e **q4** sono relativi al link precedente (0 = allineati; positivo = alza il link successivo).
- **pitch** del TCP = q2 + q3 + q4 (0 = orizzontale, −90° = utensile verso il basso).
- **roll** = q5.
- Gomito **su**: il gomito sta sopra la retta spalla–polso (q3 ≤ 0). È il default. Il prototipo originale
  chiamava "gomito in su" la soluzione opposta.

Una posa raggiungibile da un braccio a 5 DOF ha l'asse di approccio nel piano del braccio: si specifica con
`x y z pitch roll` (lo yaw è implicito, `atan2(y, x)`).

## Cinematica inversa (forma chiusa)

1. `q1 = atan2(y, x)` (singolare se il TCP è sull'asse della base).
2. Centro del polso nel piano del braccio: `rw = r − a1 − d5·cos(pitch)`, `zw = z − d1 − d5·sin(pitch)`.
3. Due link (a2, a3) verso (rw, zw): `cos q3 = (rw² + zw² − a2² − a3²) / (2·a2·a3)`, segno di `sin q3` scelto
   dalla configurazione del gomito.
4. `q2 = atan2(zw, rw) − atan2(a3·sin q3, a2 + a3·cos q3)`, `q4 = pitch − q2 − q3`, `q5 = roll`.
5. Verifica dei limiti dei giunti.

Test su PC: `firmware/test/host/test_kinematics.c` (andata e ritorno FK↔IK su griglia per entrambe le
configurazioni, casi noti, irraggiungibile, singolarità, limiti).

## Dai giunti ai motori

```
passi_per_rad = passi_giro × microstep × rapporto / 2π
passi = round(± q · passi_per_rad)          (segno: `invert`)
```

Il riferimento (`zero`) dichiara che la posizione attuale corrisponde alla posa di parcheggio.

## Parametri da misurare

Tutti in `firmware/components/robot/robot_config.c`. I valori attuali sono segnaposto.

| Parametro | Valore attuale | Da misurare/decidere |
|---|---|---|
| d1, a1, a2, a3, d5 [mm] | 100, 0, 150, 150, 60 | dal CAD (assi dei giunti) |
| passi/giro | 200 | 200 per motori da 1,8° |
| microstep | 16 | come impostato dai jumper MS1/MS2 del TMC2209 |
| rapporto | 40 (per tutti) | rapporto reale di ogni cicloidale (e cinghie) |
| invert | no | se un giunto va al contrario |
| limiti [°] | vedi file | dai finecorsa meccanici, con margine |
| v max, a max | 45–60 °/s, 90–120 °/s² | partire bassi e salire con le prove |
| posa di parcheggio [°] | 0, 90, −90, 0, 0 | una posa stabile e riproducibile (dima o tacche) |

Nota: con 40.000 passi/s al massimo e 355,6 passi/° (200×16×40) la velocità massima di un giunto è ~112 °/s;
il firmware limita comunque al 90 % e segnala i limiti configurati troppo alti.
