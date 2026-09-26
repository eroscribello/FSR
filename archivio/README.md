# Archivio

Strumenti che hanno **finito il loro lavoro**. Non sono rotti e non sono
inutili: hanno risposto a una domanda, la risposta e' registrata qui sotto, e
tenerli nella cartella principale confonderebbe chi cerca gli script delle
campagne.

Questa cartella **non va messa sul path MATLAB**. Se serve rilanciare uno di
questi strumenti, si aggiunge il path per quella sessione:

```matlab
addpath('archivio');   % solo per la sessione corrente
```

Nessuno di questi file e' chiamato da codice attivo: le uniche ricorrenze dei
loro nomi nella repo sono in commenti. E' stato verificato prima di spostarli.

---

## L'indagine sul fallimento a 2x

Quattro strumenti, una domanda: perche' a 2x il robot cammina **indietro**
(frazione del task da -26% a -69% in tutte le celle della griglia di
taratura, con contatto apparentemente buono)?

**Risposta: il tripode non regge l'andatura.** I giunti eseguono la corsa
comandata - 71 mm su 60 nominali, identici a 1x, 1.5x e 2x - ma il corpo
rimbalza di 81 mm su 154 di altezza di appoggio, resta sotto tre piedi il 61%
del tempo e perde l'aggancio di fase col contatto (accordo 51%, cioe'
testa-o-croce). Non e' un difetto del banco: e' il limite del controllore
cinematico comandato in posizione.

Poi `limite_velocita.m` ha individuato **1.20x** come ultima cella con moto del
corpo stabile (resta attivo: va rilanciato se cambia la geometria o la
taratura). Non si chiama piu' "limite di C1": la condizione sul tripode era
tarata sulle forze ricostruite e con i sensori boccia anche il nominale. Vedi
`common/phantomx_config.m`, sezione T2.

### `prova_filtri.m`
Sweep di `tau`, il tempo dei diciotto filtri del primo ordine costruiti
dall'InitFcn, per capire se ritardassero i comandi a 2x.

Esito: **lo sweep era vuoto per costruzione**. `sys_filter` esiste, ha 18
stati e i suoi poli seguono `tau` alla cifra, ma nessun blocco del modello lo
legge. Cinque valori di `tau` davano risultati identici bit per bit.

Lezione tenuta: la prima versione verificava che la *variabile* `tau`
cambiasse, non che il *filtro* cambiasse. Controllare l'ingresso di una
catena non dice niente sull'uscita, e una verifica insufficiente e' peggio di
nessuna verifica perche' autorizza la conclusione sbagliata.

### `trova_filtri.m`
Censimento di chi usa `sys_filter` o `tau` nel modello, su 2085 blocchi,
commentati compresi, piu' i blocchi dinamici (Transfer Fcn, Rate Limiter,
Unit Delay...) che filtrerebbero con i coefficienti scritti a numero senza
nominare nessuna variabile.

Esito: **`sys_filter` e' un residuo.** L'InitFcn lo costruisce e lo stampa;
il modello lo ignora.

### `causa_2x.m`
Doveva distinguere fra scivolamento invertito e traiettoria di swing non
chiusa, misurando lo spostamento del piede durante l'appoggio.

Esito: **ha dato due verdetti sbagliati prima di dare quello giusto**, e
vale piu' come lezione sul metodo che come strumento.

- v1 misurava per segmento di contatto *osservato*: quando l'appoggio si
  frammenta i segmenti si accorciano, quindi lo spostamento per segmento cala
  anche se lo scivolamento non cala. Quantita' confondente.
- v2 passava alle finestre *comandate* e usava l'identita'
  `Dcorpo = slip - sweep` come guardia. Ma quell'identita' e' algebra - viene
  da `p_piede = p_corpo + R*p_piede_corpo` - quindi **chiude per qualunque
  finestra**: valida il cambio di frame, non l'aggancio della fase. Con
  l'accordo di fase al 51% ha concluso "i giunti non inseguono" leggendo
  `sweep = -1.98 mm`, che e' quello che si ottiene quando la finestra
  straddia la transizione appoggio/volo e somma un pezzo di `-S` con un pezzo
  di `+S`.
- v3 ha la guardia giusta: con la finestra toccata sotto il 70% non emette
  quel verdetto e rimanda a `inseguimento`.

### `inseguimento.m`
La misura che ha chiuso la questione, ed e' chiusa perche' **non dipende
dalla fase**: l'escursione picco-picco del piede nel frame del corpo vale `S`
qualunque sia l'istante in cui inizia l'appoggio.

Esito: 72.3 / 71.7 / 71.0 mm a 1x / 1.5x / 2x. I giunti inseguono sempre, e
il verdetto di `causa_2x` era un artefatto della finestra.

Legge anche il modo di attuazione dei giunti, che e' il controllo da fare per
primo: se il moto e' imposto, un ritardo di inseguimento e' impossibile per
costruzione e l'ipotesi si esclude senza simulare niente.

Difetto noto e non corretto: l'escursione misurata e' il 118-120% di `S` a
tutte le velocita', non il 100%. E' un offset sistematico, probabilmente
l'origine della cinematica diretta (`adatta_simscape` dichiara che puo'
essere sfalsata di qualche mm). Non tocca la conclusione perche' e' costante
sui tre punti, ma **non e' spiegato**.

---

## Cosa NON e' stato archiviato, e perche'

- ~~`mappa_contatti.m` e `quota_terreno.m` erano chiamati da `applica_terreno`~~
  **[AGGIORNATO 21/9]** Non piu': la riscrittura del collega indirizza i blocchi
  per nome (`Contact_pavimento`, `Solid_Ostacolo3`, ...) e la mappa letta dal
  modello non serve. Nessun file li chiama: sono da archiviare anche loro.

## Da quando i blocchi hanno un nome

`mappa_contatti` risolveva a runtime una permutazione fra contatti ed
etichette - `Force6` toccava `ostacolo`, `Force2` toccava `ostacolo2` - perche'
i blocchi si chiamavano `Spatial Contact Force1..8` e nessuno sapeva quale
fosse quale. Il collega ha rinominato i blocchi nel modello, uno per elemento
per piede, e la permutazione e' sparita: noi avevamo curato il sintomo, lui ha
tolto la causa.

`quota_terreno` misurava dagli STL la quota delle superfici, e ne era uscita la
correzione `ost_dz = 0.025` (pavimento imperfetto meno liscio). La correzione
e' ancora in `cfg`, ma il nuovo catalogo ha un solo `pavimento`: se il
pavimento imperfetto non c'e' piu', quel valore va riverificato - a occhio in
T5, l'ostacolo deve poggiare.
- `taratura_T2.m` resta attivo: va rilanciato se cambia la geometria delle
  zampe o il terreno.
- `limite_velocita.m` resta attivo per la stessa ragione.
- ~~`abilita_log.m`, `verifica_log.m`, `cerca_log.m` sono chiusi come
  indagine~~ **[AGGIORNATO 26/9] Non lo sono piu': `abilita_log` e' tornato
  uno strumento attivo.** `valida_soglia.m` lo chiama per accendere `Fleg` e
  avere la forza normale al piede, che e' la VERITA' contro cui si misura il
  flag di contatto di C2. Senza, quella validazione non si puo' fare. Restano
  in root tutti e tre (`abilita_log` chiama `cerca_log`, `verifica_log`
  controlla che l'accensione abbia attecchito).
  Resta vero che le forze non sono ottenibili in tutte le condizioni: su T6 i
  sensori chiudono al 41-65% del peso, vedono il pavimento e non i piedi
  sugli ostacoli. E' un limite del modello, non dello strumento.

---

## [26/9] Il secondo giro di archiviazione

Ventitre' file, tutti verificati orfani con un grafo delle chiamate costruito
su root, `common/`, `simscape/` e `metriche/`, escludendo le occorrenze nei
commenti. Sei gruppi.

### Indagine S0 e coppie statiche
`prova_S0` `s0_vuoto` `prova_coppie` `posa_iniziale` `controlla_copia`
`corpi_degeneri` `ispeziona_giunti` `tau_misurato` `tau_statico`
`crea_modello_mpc`

Servivano a far stare in piedi il robot a coppie costanti sulla copia del
modello con i giunti attuati in coppia, primo passo verso l'MPC.

Esito: **S0 e' un equilibrio instabile e il test e' stato ritirato**, non
riparato (`docs/piano_mpc_simscape.md` §3b, marcata `[RITIRATO 24/9]`). A
coppie di giunto fisse la forza al piede e' `f = -(J')^-1 tau`, e `J` dipende
dalla configurazione: se il corpo sale la zampa si estende, il braccio si
accorcia, la spinta cresce e il corpo sale ancora. Diverge in entrambe le
direzioni, e lo smorzamento combatte la velocita', non il segno del guadagno.

Le tre cose che S0 doveva verificare sono state verificate meglio altrove, e
quei risultati restano validi: `prova_coppie` ha confermato il percorso
`Constant -> convertitori -> giunti` con rapporto **1.000 su tutti e 18** i
giunti - ed e' il motivo per cui oggi sappiamo che un `tau = -J'f` arriverebbe
davvero ai giunti, se si scrivesse un C3 in coppia; `tau_misurato` ha letto le
coppie statiche vere da `torque_sens`; `posa_iniziale` ha mostrato che base e
copia partono dalla stessa posa, sei piedi a terra a 2.788 N.

### Indagine sulle inerzie
`prova_inerzie` `controllo_inerzie`

Hanno misurato l'effetto della correzione delle inerzie URDF su T2 e T6, C1 e
C2. Esito e numeri in `docs/piano_confronto.md` §9, insieme alle due prove che
quelle dell'URDF sono sbagliate: raggio di girazione di 1.79 m per un corpo da
25 cm, e ventiquattro link con inerzia identica.

La correzione vive in `applica_inerzie.m`, che resta attivo. `sistema_results`
nomina `prova_inerzie*.csv`, cioe' i CSV, non lo script.

### Geometria, terreno, verifiche una tantum
`confronta_terreno` `complanarita` `analizza_forze` `catena_gamba`
`misura_quota` `verifica_marcia`

Risposte gia' registrate: catalogo del terreno e pose degli ostacoli
(`confronta_terreno`, usato nell'indagine del 15 settembre), complanarita' dei
piedi, catena cinematica della gamba, quota d'appoggio, marcia nominale.
`complanarita` chiama `analizza_forze`, che infatti viene con lui.

### Modifiche strutturali al `.slx`
`setup_modello` `setup_terreno_param`

Gia' applicate. **`setup_modello` e' l'unico script del progetto che fa
`save_system`**, ed e' la ragione principale per cui sta qui: tutto il resto
lavora in memoria per principio - `applica_terreno`, `applica_inerzie`,
`allinea_stimatore` - e un file che salva il modello del collega non deve
stare nella cartella dove si lanciano le campagne.

### Imbardata e beccheggio
`curva_imbardata` `origine_beccheggio`

Indagini chiuse. Quello che serve alla campagna T3 e' rimasto in root:
`script_T3` chiama `diagnosi_imbardata` e `tasso_imbardata`.

### Scratch
`Graph_generator`

Cinque righe di `plot` del collega, senza intestazione: disegna `c_lf` e
`z_lf`, cioe' il flag di contatto e la profondita' comandata della zampa
anteriore sinistra. Non si cancella perche' non e' nostro.

---

## [26/9] Una nota che vale piu' di un file: `sys_filter` e' inerte

E' gia' scritto sopra, nella sezione sull'indagine del fallimento a 2x, ma va
ripetuto perche' ci sono ricascato il 25/9. L'`InitFcn` del modello costruisce
diciotto filtri del primo ordine con costante di tempo 50 ms e li stampa a
ogni apertura. Chi li vede per la prima volta pensa a un ritardo nascosto
nella catena delle coppie - ed e' un'ipotesi che sembra ottima, perche'
spiegherebbe un flag di contatto che scatta in anticipo.

**Non lo e'.** `trova_filtri.m` ha censito 2085 blocchi, compresi i commentati
e quelli con i coefficienti scritti a numero: nessuno legge `sys_filter`.
`prova_filtri.m` ha spazzato cinque valori di `tau` ottenendo run identiche
bit per bit. E' un residuo.

Prima di dedicargli mezza giornata, leggere qui.
