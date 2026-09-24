# Cinematica

Geometria misurata su `hardware/cad/robot.step`. Codice: `firmware/components/kinematics`, test:
`firmware/test/host/test_kinematics.c`.

## Struttura

| Giunto | Tipo | Asse |
|---|---|---|
| J1 base | yaw | verticale |
| J2 spalla | pitch | orizzontale, perpendicolare al piano del braccio |
| J3 gomito | pitch | parallelo a J2 |
| J4 avambraccio | **roll** | asse dell'avambraccio |
| J5 polso | pitch | perpendicolare all'avambraccio, incontra l'asse di J4 nel **centro polso W** |

I link sono montati di fianco ai giunti: il braccio è spostato di 67,4 mm lungo l'asse della spalla e l'avambraccio
torna in asse al gomito, quindi l'avambraccio resta sempre nel piano verticale della base. Il gripper (3 dita) è
spostato di **67,4 mm lungo l'asse di J5** e sporge di **155,4 mm** oltre il polso.

```
            TCP (punta delle dita)
             │ 155,4
   67,4 ─────● asse J5 (pitch polso)        W = centro polso
             │ J4 roll attorno all'avambraccio
             │ a3 = 151,8
             ● J3 (gomito)
             │ a2 = 159,2
             ● J2 (spalla)
             │ d1 = 104,8
            ═╧═ J1 (base)
```

| Parametro | Valore | Significato |
|---|---|---|
| `d1` | 104,8 mm | piano di base → asse spalla |
| `a2` | 159,2 mm | asse spalla → asse gomito |
| `a3` | 151,8 mm | asse gomito → centro polso |
| `tool_offset` | 67,4 mm | TCP lungo l'asse di J5 |
| `tool_length` | 155,4 mm | TCP lungo la direzione di avvicinamento (punta delle dita) |

## Convenzioni

- Terna di base: z in alto, x in avanti con J1 = 0; J1 positivo antiorario visto dall'alto.
- **q2** = angolo del braccio rispetto all'orizzontale (positivo in su). **q3** relativo al braccio
  (0 = dritto, positivo alza l'avambraccio).
- **q4** = roll dell'avambraccio; con q4 = 0 l'asse di J5 è la normale al piano del braccio (lato del gripper).
- **q5** = pitch del polso; con q4 = q5 = 0 la pinza è allineata all'avambraccio.
- Posa del CAD (braccio dritto in verticale) = `0 90 0 0 0` → TCP a (0; 67,4; 571,2).
- Gomito **su** = gomito sopra la retta spalla–polso (q3 ≤ 0), default.

## Cosa si controlla

Con 5 giunti si controllano la **posizione del TCP** e la **direzione di avvicinamento** della pinza
(`pitch`: −90° = verso il basso; `yaw`: azimut). La rotazione della pinza attorno al proprio asse ne è una
conseguenza (il gripper a 3 dita è quasi simmetrico).

## Cinematica inversa

1. Il centro polso W sta nel piano del braccio: da W si ottengono q1 e (q2, q3) in forma chiusa (2 link).
2. Dalla direzione di avvicinamento nel piano del braccio si ottengono q4 e q5 in forma chiusa (due soluzioni:
   si sceglie quella con q4 più vicino alla posizione attuale).
3. W dipende da q4 per via dell'offset di 67,4 mm: si risolve con Newton sul centro polso, partendo dal centro
   polso della posizione attuale; se fallisce (vicino a singolarità) raffinamento ai minimi quadrati sui giunti.
4. Il risultato è sempre verificato con la cinematica diretta.

Prestazioni (test su PC): 100 % delle pose raggiungibili risolte partendo dalla posizione attuale, stessa
configurazione ritrovata nel 99,7 % dei casi; da zero risolve tutti i punti raggiungibili con pinza verso il basso.

Singolarità: braccio tutto disteso (q3 ≈ 0), centro polso sull'asse della base, polso dritto (q5 ≈ 0, q4
indeterminato: viene mantenuto quello attuale).

## Dai giunti ai motori

```
passi_per_rad = passi_giro × microstep × rapporto / 2π      (200 × 16 × 20 = 64000 passi/giro = 177,8 passi/°)
passi = round(± q · passi_per_rad)                           (segno: `invert`)
```

## Parametri ancora da verificare

In `firmware/components/robot/robot_config.c`:

| Parametro | Valore attuale | Da fare |
|---|---|---|
| microstep | 16 | come impostato dai jumper MS1/MS2 (H10/H9) |
| rapporto | 20:1 tutti i giunti | confermato dall'autore |
| invert | no | se un giunto va al contrario |
| limiti [°] | vedi file | dai fine corsa meccanici, con margine |
| v max, a max | 45–60 °/s, 90–120 °/s² | partire bassi e salire con le prove |
| posa di parcheggio [°] | 0, 90, −90, 0, 0 | una posa stabile e riproducibile (dima o tacche) |
