# Architettura del firmware

## Obiettivi

- Impulsi STEP precisi e deterministici, indipendenti da WiFi, log e comandi.
- Tutti i giunti coordinati: partono e arrivano insieme, ognuno entro i propri limiti.
- La matematica (cinematica, profili, generazione dei passi) è C puro, testata sul PC.
- Stato sicuro all'accensione: driver disabilitati, nessun movimento finché non lo chiedi.

## Task, core e flusso dei dati

```
 Core 0 (non real-time)                       Core 1 (real-time)
 ┌────────────────────────┐  coda comandi ┌─────────────────────────┐ coda segmenti ┌──────────────────────┐
 │ comms: console seriale │ ────────────▶ │ task motion (1 kHz,     │ (lock-free)──▶│ ISR stepgen 40 kHz   │
 │ robot: stati, e-stop,  │               │ svegliato dall'ISR):    │               │ GPTimer, solo interi,│
 │ gripper, LED; WiFi     │ ◀──────────── │ s(t) → q(t) → step      │               │ IRAM, registri GPIO  │
 └────────────────────────┘ stato         └─────────────────────────┘               └──────────────────────┘
```

| Elemento | Dove | Frequenza | Note |
|---|---|---|---|
| ISR `stepgen` | core 1, priorità 3 | 40 kHz (25 µs) | intera, in IRAM, gira anche a cache disattivata |
| Task `motion` | core 1, priorità 23 | 1 kHz (un segmento) | svegliato dall'ISR a ogni segmento consumato |
| Console `comms` | core 0 | a richiesta | risposte scritte in un colpo solo |
| WiFi, LED, gripper | core 0 | — | |

L'interrupt del GPTimer viene allocato sul core che registra le callback: per questo `stepgen_init()` è
chiamata dal task `motion`, già vincolato al core 1.

## Generazione dei passi (`stepgen`)

Il moto è diviso in **segmenti** da 1 ms (40 tick). Ogni segmento dice quanti passi (con segno) deve fare
ciascun asse. L'ISR li distribuisce uniformemente con un DDA intero:

```
acc += n            (n = |passi| del segmento, T = 40 tick)
if acc >= T:  un passo, acc -= T
```

Partendo da `acc = T/2` si ottengono esattamente `n` passi per segmento, spaziati in modo uniforme (±1 tick).
Massimo un passo per tick, cioè 40.000 passi/s per asse.

Sequenza di ogni tick:
1. alza gli STEP calcolati al tick precedente (una scrittura su `GPIO.out_w1ts` per i GPIO 0–31 e una su
   `GPIO.out1_w1ts` per i GPIO 32–33);
2. calcola il tick successivo, caricando il segmento seguente quando serve;
3. attende la larghezza minima dell'impulso (1 µs, misurata con CCOUNT) e abbassa gli STEP;
4. scrive i DIR cambiati: il fronte di salita successivo arriva almeno un tick dopo (setup time);
5. notifica il task `motion` a fine segmento.

Se la coda si svuota a metà di un movimento conta un **underrun**; un segmento con troppi passi viene
**rifiutato**. Un watchdog nel task `motion` controlla che i tick avanzino e riarma il timer se un allarme
va perso (`stalls`).

## Pianificazione (`motion`)

Un movimento MOVEJ è una retta nello spazio giunti parametrizzata da `s ∈ [0,1]`:
`q(s) = q0 + Δq·s`. I limiti del parametro sono

```
ṡmax = minᵢ vᵢ / |Δqᵢ|        s̈max = minᵢ aᵢ / |Δqᵢ|
```

quindi il giunto più "lento" detta i tempi e tutti arrivano insieme. `s(t)` è un profilo trapezoidale (o
triangolare) valutato **in forma chiusa** a ogni ms: niente integrazione numerica, niente deriva, arrivo
esatto sul target. Ogni ms: `s(t) → q(t) → passi assoluti (arrotondati) → delta → segmento`.

`stop` sostituisce il profilo con una decelerazione lungo lo stesso percorso; `estop` interrompe subito
(possibili passi persi ad alta velocità) e riallinea lo stato dalle posizioni reali dello stepgen.

Il task tiene in coda ~20 segmenti (20 ms): è la latenza di `stop` e la tolleranza a ritardi del task.

## Stati del robot (`robot`)

```
DISABLED ──enable──▶ ENABLED ──zero──▶ READY ⇄ (in movimento)
    ▲                   │                 │
    └──────disable──────┴─────────────────┘      ESTOP (da qualunque stato) ──reset──▶ stato precedente
```

- All'avvio: `DISABLED`, driver spenti, non riferito.
- Senza finecorsa il riferimento è manuale: metti il braccio nella posa di parcheggio e dai `zero`.
- I movimenti assoluti richiedono `READY`; il `jog` relativo funziona anche non riferito, a velocità ridotta.
- `disable` perde il riferimento (il braccio può muoversi liberamente).

## Regole

- Unità interne: mm e radianti. I gradi compaiono solo nel protocollo.
- Niente scritture in flash (NVS) durante il moto: con la cache disattivata il task `motion` si ferma e
  la coda di segmenti si svuoterebbe.
- La mappa pin sta solo in `components/board`. La configurazione meccanica sta solo in
  `components/robot/robot_config.c`.
- Codice, commenti e commit in inglese; documentazione in italiano.
