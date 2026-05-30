[🇺🇸 English](README.md) | [🇨🇳 中文](docs/README_CN.md) | [🇹🇼 繁體中文](docs/README_TW.md) | [🇪🇸 Español](docs/README_ES.md) | [🇫🇷 Français](docs/README_FR.md) | [🇷🇺 Русский](docs/README_RU.md) | [🇸🇦 العربية](docs/README_AR.md) | [🇩🇪 Deutsch](docs/README_DE.md) | [🇯🇵 日本語](docs/README_JA.md) | [🇰🇷 한국어](docs/README_KO.md) | [🇮🇹 Italiano](docs/README_IT.md) | [🇵🇹 Português](docs/README_PT.md)

---

# TCP UCP v1.0 (Protocole de Communication Universel)

Module de contrôle de congestion TCP pour environnements VPS à bande passante partagée, combinant la machine d'état BBRv1 avec un filtre de Kalman pour l'estimation du délai de propagation.

## Design Principles

Congestion control algorithms must balance throughput, latency, fairness, and loss tolerance. UCP takes a pragmatic approach:

1. BBRv1 provides a proven foundation. State machine, pacing, cycle gains, STARTUP/DRAIN/PROBE_BW/PROBE_RTT — UCP adopts these mechanisms without modification.

2. The Kalman filter improves estimation accuracy. Separating true propagation delay from queuing delay and jitter yields a more accurate min_rtt estimate, enabling tighter BDP calculation, better-calibrated CWND, and more stable pacing.

3. Inter-algorithm dynamics follow standard TCP competitive equilibrium. UCP does not artificially limit its send rate in response to queue detected from external flows. Gain decay (queue-based probe reduction) is available as opt-in via ucp_cycle_decay_mask but disabled by default to preserve full probe intensity.

4. Intra-UCP fairness is actively maintained. Kalman convergence ensures UCP flows on the same host share a consistent min_rtt estimate, eliminating the winner-takes-all feedback loop that causes severe unfairness in pure BBR multi-flow deployments.

## Aperçu de l'Algorithme

TCP UCP implémente un module de contrôle de congestion côté émetteur pour le noyau Linux sous forme d'un module chargeable `tcp_ucp.ko`. La fonction de contrôle de congestion `ucp_main()` est invoquée à chaque ACK depuis `tcp_ack()`, recevant une structure `rate_sample` qui contient des échantillons de bande passante et de RTT du noyau ainsi que des compteurs de livraison et de perte. L'algorithme opère dans deux régimes temporels : un **chemin rapide par-ACK** qui met à jour l'état des mesures et calcule les objectifs instantanés de pacing et de fenêtre, et un **chemin lent par-round** qui évalue les conditions de transition d'état et recalcule les gains.

Le pipeline de mesure central se compose de deux composants :

1. **Filtre de bande passante maximale à fenêtre glissante** (`minmax_running_max` de `linux/win_minmax.h`) : fenêtre couvrant les dernières `ucp_bw_rt_cycle_len` (par défaut 10) rounds. Fournit l'estimation `max_bw` compatible BBR.

2. **Estimateur de délai de propagation par filtre de Kalman** : remplace le RTT minimum à fenêtre glissante de BBRv1. Un filtre de Kalman à état unique (Kalman 1960) opérant en unités virgule fixe de `ucp_kalman_scale` × µs, modélisant le délai de propagation réel comme une marche aléatoire :
   - État : `x[k] = x[k−1] + w[k]`, `w ~ N(0, Q)`
   - Observation : `z[k] = x[k] + v[k]`, `v ~ N(0, R)`

Conventions virgule fixe : `BW_UNIT = 1 << 24` pour la bande passante (segments * 2^24 / µs), `BBR_UNIT = 1 << 8 = 256` comme unité de gain adimensionnelle.

## Machine d'État

```
    ┌───> STARTUP ────┐
    │       │         │
    │       ▼         │
    │     DRAIN  ─────┤
    │       │         │
    │       ▼         │
    └─── PROBE_BW ────┘
    │      ^    │
    │      │    │
    │      └────┘
    │
    └─── PROBE_RTT <──┘
```

Quatre modes codés sur le champ `mode` de 2 bits dans `struct ucp` :

- **STARTUP (0)** : État initial. `pacing_gain` ≈ 2,885x (`ucp_high_gain_val`), `cwnd_gain` également 2,885x. Sondage exponentiel de bande passante.
- **DRAIN (1)** : Entré après la sortie de STARTUP. `pacing_gain` ≈ 0,347x (`ucp_drain_gain_val`), `cwnd_gain` reste à 2,885x. Vide la file accumulée pendant STARTUP.
- **PROBE_BW (2)** : État stable. Parcourt une table de gains de 256 emplacements (motif par défaut de 8 phases répété : 1,25x/0,75x/8×1,0x).
- **PROBE_RTT (3)** : Vide périodiquement le trafic en vol à `ucp_cwnd_min_target` (par défaut 4 segments) pour obtenir un échantillon RTT frais.

### STARTUP → DRAIN

Déclenché lorsque `full_bw_reached` est activé — après `ucp_full_bw_cnt` (par défaut 3) rounds consécutifs où `max_bw` n'atteint pas une croissance d'au moins `ucp_full_bw_thresh_val` (par défaut 1,25x) par rapport au pic observé précédemment. Le BDP à gain 1,0x est écrit dans `snd_ssthresh`. `qdelay_avg` est remis à zéro pour éviter que l'accumulation de file de STARTUP n'affecte PROBE_BW.

### DRAIN → PROBE_BW

Déclenché lorsque le trafic en vol estimé à EDT ≤ trafic en vol cible à gain BDP de 1,0x. **Optimisation de saut de DRAIN** : lorsque le filtre de Kalman est convergé ET `qdelay_avg` est inférieur à `ucp_drain_skip_qdelay_us` (par défaut 1000 µs), la phase DRAIN est sautée — conversion précoce vers PROBE_BW.

À l'entrée de PROBE_BW, l'indice de phase du cycle est randomisé : `cycle_idx = len − 1 − rand(ucp_probe_bw_cycle_rand)` (par défaut `len − 1 − rand(8)`), ce qui décorrèle les flux concurrents partageant un lien engorgé.

### PROBE_BW → PROBE_RTT

Déclenché lorsque l'intervalle du filtre PROBE_RTT expire — l'horodatage `min_rtt_stamp` n'a pas été mis à jour dans l'intervalle calculé. cwnd est sauvegardé dans `prior_cwnd`, le pacing est configuré pour vidanger.

### PROBE_RTT → PROBE_BW

Après que le trafic en vol chute à `ucp_cwnd_min_target` ou qu'une limite de round est observée, persiste pendant au moins `ucp_probe_rtt_mode_ms_val` (par défaut 200 ms) et au moins un round complet observé, puis sort. cwnd est restauré à au moins `prior_cwnd`, le pacing est temporairement remplacé par `ucp_high_gain_val` pour un remplissage rapide du pipe.

### Récupération et Perte

- Sur TCP_CA_Loss : `full_bw` et `full_bw_cnt` sont réinitialisés, `round_start` mis à 1, `packet_conservation` effacé à 0. Si LT BW n'est pas actif, injecte un événement de perte synthétique pour déclencher l'échantillonnage LT.
- Entrée de récupération (TCP_CA_Recovery) : `packet_conservation` activé, cwnd = en vol + acquitté.
- Sortie de récupération : restauré à `prior_cwnd`, `packet_conservation` effacé.
- `ucp_undo_cwnd()` : réinitialise `full_bw` et `full_bw_cnt` (en préservant `full_bw_reached`), efface l'état LT BW.

## Mesures Centrales

### Estimation de Bande Passante

Filtre de bande passante maximale à fenêtre glissante (`minmax_running_max` de `linux/win_minmax.h`) sur `ucp_bw_rt_cycle_len` (par défaut 10) rounds. bw instantané = `delivered × BW_UNIT / interval_us` calculé par ACK. Alimenté dans la fenêtre glissante seulement lorsqu'il n'est pas limité par l'application ou lorsque bw ≥ bw max actuel (règle BBR).

Lorsque `lt_use_bw` est actif, l'estimation de bande passante active bascule vers `lt_bw` (estimation de bande passante à long terme).

### Filtre de Kalman

Récursion de Kalman scalaire à état unique (complexité O(1)) :

```
Prédire :
  x_pred = x_est          (transition d'état identité)
  p_pred = p_est + Q      (prédiction de covariance)

Mettre à jour :
  innov   = z − x_pred    (innovation)
  K       = p_pred / (p_pred + R)   (gain de Kalman [0,1])
  x_est   = x_pred + K × innov      (mise à jour d'état)
  p_est   = (1 − K) × p_pred        (covariance postérieure)
```

**Bruit de processus adaptatif Q** :
```
Q_base   = ucp_kalman_q (par défaut 100)
q_factor = max(ucp_kalman_q_min_factor_val, min_rtt_us / ucp_kalman_q_rtt_div)
Q        = min(Q_base × q_factor, Q_base × ucp_kalman_q_scale_cap)
Q        = min(Q, ucp_kalman_q_max)
```

**Bruit de mesure adaptatif R** :
```
R = R_base + max(0, jitter_ewma − ucp_jitter_r_thresh_us) × R_base / ucp_jitter_r_scale
R = min(R, R_base × ucp_kalman_r_max_boost)
```

**Q-Boost path-change detection (confidence-gated + cooldown)**: lorsque `|innovation| > ucp_kalman_q_boost_thresh_val` (default ≈ 4 ms RTT shift) AND the filter has converged (`p_est ≤ ucp_kalman_converged_p_est_val`, default 500), `p_est` is reset to `ucp_kalman_p_est_init_val`, boosting Kalman gain toward 1.0 for rapid convergence. A cooldown of `ucp_kalman_qboost_cdwn` (default 15) samples between successive qboost events prevents runaway triggering on lossy paths with high RTT jitter.

**Porte de valeurs aberrantes** : seuil dynamique `dyn_thresh = max(outlier_ms × 1000 × scale, jitter_ewma × outlier_jitter_mult × scale)`. Appliqué seulement lorsque `p_pred ≤ ucp_kalman_converged_p_est_val`. Après `ucp_kalman_max_consec_reject` (par défaut 25) rejets consécutifs, l'échantillon suivant est forcé d'être accepté pour éviter un verrouillage auto-renforçant.

**Estimation de bruit par covariance appariée (BBR-S)** : `q_est = (1−α) × q_est + α × (K × innov)²`, `r_est = (1−β) × r_est + β × max(0, innov² − p_pred)`. Mode de combinaison : mode 0 = heuristique seulement, mode 1 = max (par défaut), mode 2 = mélange pondéré.

**Prise de contrôle de Kalman** : lorsque `x_est > 0` et `sample_cnt ≥ ucp_kalman_min_samples` (par défaut 5), `min_rtt_us` est remplacé par `x_est / ucp_kalman_scale`. `min_rtt_stamp` n'est pas mis à jour — le déclencheur d'intervalle PROBE_RTT reste indépendant.

**Plafond de marge x_est** : Le `model_rtt` dérivé de Kalman est plafonné à `min_rtt_us × (100 + ucp_kalman_xest_margin_pct) / 100` (par défaut 8%).

## Améliorations BBR

### Décroissance de Gain

Activée par le bitmap 256 bits `ucp_cycle_decay_mask[]` pour des phases spécifiques de PROBE_BW. Formule de décroissance (sur échantillon Kalman accepté) :

```
max_red       = probe_gain − BBR_UNIT
conf_scale    = mise à l'échelle inverse de p_est (BBR_UNIT à plein)
qdelay_decay  = min(max(0, qdelay_avg − qthresh) × BBR_UNIT / qscale, max_red)
                     × conf_scale / BBR_UNIT
jitter_decay  = min(max(0, jitter_ewma − jthresh) × BBR_UNIT / jscale, remaining)
                     × conf_scale / BBR_UNIT
effective     = max(probe_gain − qdelay_decay − jitter_decay, BBR_UNIT)
```

Mise à l'échelle de confiance Kalman : lorsque `p_est > ucp_kalman_converged_p_est`, la décroissance est proportionnellement réduite, évitant un recul excessif lorsque le filtre est incertain.

### Recul ECN

Conditions d'activation (toutes doivent être vérifiées) :
1. `ucp_ecn_enable_val != 0`
2. Kalman convergé (`p_est < converged`, `sample_cnt >= min_samples`)
3. `ecn_ewma > 0` (marques CE observées)
4. `qdelay_avg > ucp_ecn_qdelay_thresh_us_val` (par défaut 2000 µs)
5. Le mode n'est PAS PROBE_BW (cwnd_gain est fixe à 2x dans PROBE_BW)

Pendant les phases de sondage (`pacing_gain > BBR_UNIT`), le recul ECN est gradué par `BBR_UNIT² / pacing_gain` — ~80% de recul à sonde 1,25x, ~65% à gain STARTUP 2,89x.

Rapport de marque ECN EWMA : mis à jour aux limites de round par `ucp_ecn_ewma_retained / ucp_ecn_ewma_total` (par défaut 3/4), avec un déclin doux par-ACK de `ucp_ecn_idle_decay_num / ucp_ecn_idle_decay_den` (par défaut 31/32) sur chaque ACK sans nouvelle marque CE.

### Détection de Flux Unique

Lorsqu'UCP détecte que le flux est probablement seul sur le goulot d'étranglement (faible délai de file d'attente, faible gigue, aucune marque ECN, aucune agrégation d'ACK, aucune bande passante LT), il passe automatiquement en mode BBR pur :

- `ucp_get_model_rtt()` renvoie directement `min_rtt_us` (évitant l'estimation lissée de Kalman, qui a un léger biais positif dû au bruit de mesure unilatéral).
- `ucp_ecn_backoff()` est configurable via `ucp_alone_bypass_ecn` (défaut 1) — sur un chemin à flux unique, les marques ECN sont des faux positifs de l'AQM car il n'y a pas d'autre émetteur en compétition. Le sauter correspond au comportement ECN zéro de BBR. Mettez à 0 pour conserver le backoff ECN même en mode solo (conservateur).
- La condition LT BW (policer) est configurable via `ucp_alone_bypass_lt_bw` (défaut 1) — un chemin à flux unique n'a pas de policer, donc LT BW ne peut pas s'activer légitimement. Le sauter évite les sorties intempestives du mode solo dues à de faux déclenchements. Mettez à 0 pour le comportement strict d'origine.

Cela élimine l'écart de performance en flux unique entre UCP et BBR, tout en préservant la boucle de protection complète d'UCP (Kalman, backoff ECN, décroissance de gain, bande passante LT) pour les scénarios multi-flux.

**Hystérésis** : L'entrée nécessite `ucp_alone_confirm_rounds` (défaut 3) rounds consécutifs qualifiés — évitant les oscillations lors de brèves périodes d'accalmie dans la compétition multi-flux ("conservateur pour accélérer"). La sortie est immédiate — tout échec de qualification efface le drapeau et réinitialise le compteur de confirmation ("agressif pour freiner").

Conditions de qualification (les six doivent être satisfaites à une limite de tour) :
0. Kalman convergé (`sample_cnt >= ucp_kalman_min_samples`) — faire confiance à qdelay/jitter comme signaux de file
1. `qdelay_avg < ucp_alone_qdelay_thresh_us` (par défaut 1000 us) — file presque vide
2. `jitter_ewma < ucp_alone_jitter_thresh_us` (par défaut 2000 us) — micro-gigue d'horloge ACK uniquement
3. `ecn_ewma == 0` — aucune marque de congestion d'AQM
4. `lt_use_bw == 0` — pas en mode de débit limité détecté par le policer
5. `agg_state <= max` selon `ucp_alone_agg_state_level` (par défaut 1) — trois niveaux de rigueur d'agrégation ACK : 0 = IDLE seulement (le plus strict, zéro agrégation), 1 = ≤ SUSPECTED (par défaut, permet l'agrégation transitoire), 2 = ≤ CONFIRMED (le plus permissif, bloque uniquement l'agrégation persistante)

### Intervalle PROBE_RTT Dynamique

Mappe `p_est` de Kalman à un intervalle PROBE_RTT par connexion :

```
p_est ≤ converged :              interval = dyn_max (par défaut 30s)
p_est ≥ high (= mult × conv) :   interval = base (par défaut 10s)
converged < p_est < high :       interpolation linéaire
```

Réduit la fréquence de PROBE_RTT lorsque la confiance est élevée (`p_est` faible), diminuant la gigue de débit sur les chemins stables. Revient à l'intervalle classique de 10 secondes lorsque la confiance est faible.

**Gigue d'entrée par flux** : Pour empêcher tous les flux coexistants d'entrer simultanément en PROBE_RTT (se vidant à 4 paquets agrégés ~1.8 Mbps puis se remplissant à 2.89×), chaque flux ajoute une gigue dérivée du hachage (étalement de 0–845 ms) à son intervalle PROBE_RTT. Au maximum ~1 flux est en PROBE_RTT à tout instant, éliminant l'effondrement simultané vidange/remplissage induisant des RTO.

### Estimation de Bande Passante LT

Estimateur de borne inférieure déclenché par perte. L'intervalle d'échantillonnage couvre [4, 16] RTTs. Valide lorsque le taux de perte ≥ 5,9% (`ucp_lt_loss_thresh` par défaut 15/256). Bande passante `bw = delivered × BW_UNIT / interval_us`.

Contrairement à la moyenne simple de BBR (`(bw + lt_bw) >> 1`), UCP utilise un EMA configurable (`ucp_lt_bw_ema_num / ucp_lt_bw_ema_den`, par défaut 1/2 = 0,5) :

```
lt_bw = (bw_new × en + lt_bw × (ed − en)) / ed
```

L'activation diffère de BBR : UCP stocke `lt_bw` sur le premier intervalle valide mais NE définit PAS `lt_use_bw` ; la cohérence avec un intervalle précédent est requise — réduit la fausse activation due au bruit de mesure.

**Porte de congestion à double seuil** : Avant de définir `lt_use_bw = 1`, une vérification de file EWMA persistante (`qdelay_avg > ucp_ecn_qdelay_thresh_us_val`) ET une vérification de file instantanée basée sur SRTT (`srtt_us − min_rtt_us > ucp_lt_bw_inst_qdelay_thresh_us`, par défaut 5000 µs) sont évaluées. Lorsqu'une congestion est détectée, l'échantillonnage LT BW est abandonné. La vérification SRTT fonctionne sans allocation `ext`, fournissant un filet de sécurité contre les échecs d'allocation.

Boost de sonde LT BW (`ucp_lt_bw_probe_pct`, par défaut 10%) : amplifie `pacing_gain` par `1 + probe_pct/100` sur toutes les phases PROBE_BW. Composante de rampe : augmentation de `+1% par 8 RTTs`, plafonnée à `2 × probe_pct`.

Auto-récupération LT BW (`ucp_lt_restore_ratio_num/den`, par défaut 5/4 = 1,25x) : lorsque `max_bw > lt_bw × ratio` pendant `ucp_lt_restore_consec_acks` (par défaut 3) ACKs consécutifs, LT BW se désactive automatiquement et le sondage normal PROBE_BW reprend.

### Compensation Basée sur la Confiance d'Agrégation ACK (inspiré de BBRplus)

Ajoute une deuxième couche à porte de confiance au-dessus de l'estimateur traditionnel à double fente extra-acked.

**Quatre facteurs orthogonaux** (chacun contribue `ucp_agg_factor_weight` points, par défaut 256) :
1. Kalman convergé (`p_est < converged` + `sample_cnt >= min_samples`)
2. Pas en récupération de perte (`icsk_ca_state < TCP_CA_Recovery`)
3. RTT dans `min_rtt_us + ucp_agg_factor3_qdelay_us` (par défaut 2ms) du délai de propagation réel
4. `extra_acked` dans `ucp_agg_factor4_ratio_num/den` (par défaut 1,5x) du maximum fenêtré

**Quatre états** : INACTIF (< `ucp_agg_thresh_suspected`=256), SUSPECT (≥256), CONFIRMÉ (≥512), DE CONFIANCE (≥768).

**Couche de signal** (toujours active) : la confiance interpole linéairement le facteur d'échelle R `[r_min, r_max]`. R monte instantanément (réponse rapide), décroît à `ucp_agg_r_hysteresis`% (par défaut 75% retenu, ~4 RTTs pour revenir à la ligne de base) par RTT.

**Couche de contrôle** (`agg_state ≥ CONFIRMED`) : compensation cwnd à porte de sécurité à cinq couches :
1. Bloque si le délai de file > `ucp_agg_safety_qdelay_us` (par défaut 4ms)
2. Bloque pendant la récupération de perte
3. Bloque si cwnd > `BDP × ucp_agg_safety_bdp_mult` (par défaut 3x)
4. Bloque si en vol > cwnd sûr + objectif de segments TSO
5. Chien de garde : rétrograde CONFIRMÉ→SUSPECT après `ucp_agg_max_comp_duration` (par défaut 8) RTTs consécutifs

### Réinitialisation de qdelay_avg dans DRAIN

Lors de la transition vers DRAIN, `qdelay_avg` est remis à zéro, empêchant l'estimation de file de STARTUP de persister dans PROBE_BW.

### Adaptation du Diviseur TSO

`ucp_min_tso_segs()` ajuste le diviseur de seuil de débit en fonction de l'état de Kalman :
- Kalman convergé + `jitter_ewma < 1000 µs` : diviseur réduit de moitié (8→4), rafales TSO plus grandes
- `jitter_ewma > 4000 µs` : diviseur doublé (8→16), rafales TSO plus petites pour supprimer la gigue

## Taux de Pacing et Cwnd

### Taux de Pacing

```
rate = bw × mss × pacing_gain >> BBR_SCALE      // ajustement de gain
rate = rate × USEC_PER_SEC >> BW_SCALE            // conversion en bytes/s
rate = rate × margin_div / 100                    // marge de pacing (par défaut 1%, matching BBR)
```

Les changements de taux sont appliqués immédiatement (sans lissage), comme dans BBR (Cardwell et al. 2016). Après `full_bw_reached` : tous les changements de taux sont écrits immédiatement. En STARTUP/DRAIN : seules les augmentations sont appliquées (`rate > sk_pacing_rate`).

### Cwnd

```
target = BDP(bw, gain, ext)                       // BDP de base
// bornes du trafic en vol (non-STARTUP : clamp lo~hi ; STARTUP : plancher lo seulement)
target = quantization_budget(target)              // marge TSO + round pair + bonus phase-0
target += ack_agg_bonus + agg_compensation        // compensation d'agrégation ACK

// progression cwnd
if full_bw_reached:
    cwnd = min(cwnd + acked, target)              // convergence vers la cible
else (STARTUP):
    cwnd = cwnd + acked                          // croissance exponentielle

cwnd = max(cwnd, cwnd_min_target)                 // plancher absolu 4
mode PROBE_RTT : cwnd = min(cwnd, cwnd_min_target) // trafic en vol minimum
```

## Paramètres du Module

Les paramètres sont exposés sous `/proc/sys/net/ucp/`. Les écritures déclenchent `ucp_init_module_params()` (validation + écrêtage + calcul de valeur dérivée). Les écritures de paramètres de tableau déclenchent `ucp_rebuild_gain_table()`.

### Intervalles PROBE_RTT

| Paramètre | Par défaut | Min | Max | Unité | Description |
|-----------|------------|-----|-----|-------|-------------|
| `ucp_probe_rtt_base_sec` | 10 | 1 | 86400 | s | Intervalle PROBE_RTT de base |
| `ucp_probe_rtt_max_sec` | 15 | 1 | 86400 | s | Plafond pour chemins à long RTT |
| `ucp_probe_rtt_dyn_max_sec` | 30 | 0 | 86400 | s | Intervalle dynamique max ; 0 désactive |

### Gains

| Paramètre | Par défaut | Min | Max | Description |
|-----------|------------|-----|-----|-------------|
| `ucp_cwnd_gain_num` / `ucp_cwnd_gain_den` | 2 / 1 | 0/1 | 100k | Gain cwnd de base pour PROBE_BW |
| `ucp_extra_acked_gain_num` / `ucp_extra_acked_gain_den` | 1 / 1 | 0/1 | 100k/100k | Multiplicateur de bonus d'agrégation ACK |
| `ucp_high_gain_num` / `ucp_high_gain_den` | 2885 / 1000 | 0/1 | 100k | Gain STARTUP (≈2,885x) |
| `ucp_drain_gain_num` / `ucp_drain_gain_den` | 347 / 1000 | 0/1 | 100k | Gain DRAIN (≈0,347x) |
| `ucp_inflight_low_gain_num` / `ucp_inflight_low_gain_den` | 125 / 100 | 0/1 | 100k | Borne inférieure du trafic en vol (1,25x BDP) |
| `ucp_inflight_high_gain_num` / `ucp_inflight_high_gain_den` | 200 / 100 | 0/1 | 100k | Borne supérieure du trafic en vol (2,0x BDP) |
| `ucp_gain_num[i]` / `ucp_gain_den[i]` | Motif BBRv1 (256 emplacements) | 0/1 | — | Gain de pacing par emplacement |
| `ucp_cycle_decay_mask[8]` | 0 (tous zéro) | 0 | 0x7FFFFFFF | Bitmap de décroissance 256 bits |
| `ucp_probe_bw_up_limit` | 0 | 0 | 1 | Sortie limitée de sonde montant (0=désactivé) |

### Kalman de Base

| Paramètre | Par défaut | Min | Max | Description |
|-----------|------------|-----|-----|-------------|
| `ucp_kalman_q` | 100 | 0 | 100k | Bruit de processus de base Q |
| `ucp_kalman_r` | 400 | 0 | 100k | Bruit de mesure de base R |
| `ucp_kalman_p_est_max` | 1 000 000 | 1 | 100 M | p_est maximum absolu |
| `ucp_kalman_converged_p_est` | 500 | 1 | 1 M | Seuil de convergence |
| `ucp_kalman_p_est_init` | 1000 | 1 | 10 M | p_est initial |
| `ucp_kalman_p_est_floor` | 10 | 1 | 100k | Plancher p_est |
| `ucp_kalman_scale` | 1024 | 64 | 1 048 576 | Échelle virgule fixe (puissance de deux) |
| `ucp_kalman_min_samples` | 5 | 3 | 20 | Échantillons min avant prise de contrôle |
| `ucp_kalman_outlier_ms` | 5 | 0 | 10000 | ms | Seuil de base des valeurs aberrantes |
| `ucp_kalman_q_boost_mult` | 4 | 1 | 10000 | Multiplicateur Q-boost |
| `ucp_kalman_q_boost_ms` | 1 | 0 | 5000 | ms | Constante de temps Q-boost |
| `ucp_kalman_qboost_cdwn` | 15 | 1 | 255 | samples | Q-boost cooldown |
| `ucp_kalman_q_max` | 2000 | 1 | 100k | Plafond Q |
| `ucp_kalman_q_scale_cap` | 20 | 1 | 10000 | Limite d'échelle Q |
| `ucp_kalman_max_consec_reject` | 25 | 1 | 1000 | Max de rejets consécutifs avant acceptation forcée |
| `ucp_rtt_sample_max_us` | 500000 | 1 | 10 M | µs | Plafond RTT de Kalman |
| `ucp_kalman_r_max_boost` | 8 | 1 | 1000 | Multiplicateur de boost max R |
| `ucp_kalman_rtt_dyn_mult` | 2 | 1 | 100 | Multiplicateur de plafond dynamique RTT |
| `ucp_kalman_q_rtt_div` | 1000 | 1 | 1 M | Diviseur RTT d'adaptation Q |
| `ucp_kalman_probe_band_mult` | 4 | 1 | 32 | Multiplicateur de bande de transition PROBE_RTT |

### Kalman Suppléments (type num/den)

| Paramètre | Par défaut | Plage | Description |
|-----------|------------|-------|-------------|
| `ucp_kalman_outlier_jitter_mult_num/den` | 4 / 1 | 0-1000 / 1-100k | Multiplicateur de gigue pour valeurs aberrantes |
| `ucp_kalman_q_min_factor_num/den` | 10 / 1 | 0-1000 / 1-100k | Facteur min Q |
| `ucp_kalman_p_est_init_rtt_div_num/den` | 10 / 1 | 1-100k / 1-100k | Diviseur RTT d'initialisation p_est |

### Estimation de Bruit BBR-S

| Paramètre | Par défaut | Plage | Description |
|-----------|------------|-------|-------------|
| `ucp_kalman_noise_alpha_num/den` | 1 / 10 | 0-100 / 1-100k | Taux d'apprentissage estimation Q |
| `ucp_kalman_noise_beta_num/den` | 1 / 10 | 0-100 / 1-100k | Taux d'apprentissage estimation R |
| `ucp_kalman_noise_mode` | 1 | 0-2 | Mode de combinaison (0=off, 1=max, 2=moyenne pondérée) |
| `ucp_kalman_q_est_max` | 1 000 000 000 | 1-2 Mds | Borne supérieure estimation Q |
| `ucp_kalman_r_est_max` | 1 000 000 000 | 1-2 Mds | Borne supérieure estimation R |
| `ucp_kalman_q_est_floor` / `r_est_floor` | 1 | 1-100k | Borne inférieure par estimation |

### Décroissance de Gain (Sondage)

| Paramètre | Par défaut | Plage | Unité | Description |
|-----------|------------|-------|-------|-------------|
| `ucp_qdelay_probe_thresh_us` | 5000 | 0-100k | µs | Seuil de décroissance qdelay |
| `ucp_qdelay_probe_scale_us` | 20000 | 1-100k | µs | Échelle de décroissance qdelay |
| `ucp_jitter_probe_thresh_us` | 4000 | 0-100k | µs | Seuil de décroissance de gigue |
| `ucp_jitter_probe_scale_us` | 16000 | 1-100k | µs | Échelle de décroissance de gigue |

### R Adaptatif (Piloté par la Gigue)

| Paramètre | Par défaut | Plage | Unité | Description |
|-----------|------------|-------|-------|-------------|
| `ucp_jitter_r_thresh_us` | 2000 | 0-100k | µs | Seuil de gigue pour augmentation de R |
| `ucp_jitter_r_scale` | 8000 | 1-100k | — | Diviseur d'échelle d'augmentation R |

### ECN

| Paramètre | Par défaut | Plage | Description |
|-----------|------------|-------|-------------|
| `ucp_ecn_enable` | 1 | 0-1 | Interrupteur général ECN |
| `ucp_ecn_backoff_num` / `ucp_ecn_backoff_den` | 20 / 100 | 0-100 / 1-100k | Fraction de recul ECN |
| `ucp_ecn_qdelay_thresh_us` | 2000 | 0-100k | µs | Seuil qdelay ECN |
| `ucp_ecn_ewma_retained` / `ucp_ecn_ewma_total` | 3 / 4 | 0-100 / 1-100k | Poids EWMA ECN |
| `ucp_ecn_idle_decay_num` / `ucp_ecn_idle_decay_den` | 31 / 32 | 1-100k | Déclin ECN inactif |

### min_rtt

| Paramètre | Par défaut | Plage | Description |
|-----------|------------|-------|-------------|
| `ucp_minrtt_fast_fall_cnt` | 3 | 0-3 | Compteur de chute rapide |
| `ucp_minrtt_fast_fall_div` | 4 | 1-256 | Diviseur de seuil de chute rapide |
| `ucp_minrtt_sticky_num` / `ucp_minrtt_sticky_den` | 75 / 100 | 0-1000 / 1-100k | Ratio de chute persistante |
| `ucp_minrtt_srtt_guard_num` / `ucp_minrtt_srtt_guard_den` | 90 / 100 | 0-1000 / 1-100k | Ratio de garde SRTT |

### Bande Passante LT

| Paramètre | Par défaut | Plage | Description |
|-----------|------------|-------|-------------|
| `ucp_lt_intvl_min_rtts` | 4 | 1-127 | RTTs | Longueur min d'intervalle |
| `ucp_lt_intvl_max_mult` | 4 | 1-32 | Multiplicateur de timeout d'intervalle |
| `ucp_lt_loss_thresh` | 15 | 1-65535 | BBR_UNIT | Ratio de perte min |
| `ucp_lt_bw_ratio_num` / `ucp_lt_bw_ratio_den` | 1 / 8 | 0-100k / 1-100k | Tolérance relative |
| `ucp_lt_bw_diff` | 500 | 0-100k | bytes/s | Tolérance absolue |
| `ucp_lt_bw_max_rtts` | 48 | 1-4094 | RTTs | RTTs actifs max de LT BW |
| `ucp_lt_bw_probe_pct` | 10 | 0-100 | % | Boost de sonde LT BW |

### Auto-Récupération LT

| Paramètre | Par défaut | Plage | Description |
|-----------|------------|-------|-------------|
| `ucp_lt_restore_ratio_num` / `ucp_lt_restore_ratio_den` | 5 / 4 | 0-100k / 1-100k | Ratio de déclenchement de récupération |
| `ucp_lt_restore_consec_acks` | 3 | 1-31 | Nombre d'ACKs consécutifs de déclenchement |

### Confiance d'Agrégation ACK

| Paramètre | Par défaut | Plage | Description |
|-----------|------------|-------|-------------|
| `ucp_agg_enable` | 1 | 0-1 | Interrupteur général |
| `ucp_agg_confidence_thresh` | 512 | 0-10000 | Seuil de confiance de compensation cwnd |
| `ucp_agg_max_comp_ratio` | 75 | 0-100 | % du BDP | Plafond de compensation cwnd |
| `ucp_agg_max_comp_duration` | 8 | 1-128 | RTTs | Timeout du chien de garde |
| `ucp_agg_r_hysteresis` | 75 | 0-100 | % | Déclin d'hystérésis R |
| `ucp_agg_r_multiplier_min` / `ucp_agg_r_multiplier_max` | 256 / 2048 | 1-10000 | Plage d'échelle R (256=1x) |
| `ucp_agg_factor3_qdelay_us` | 2000 | 0-100k | µs | Marge qdelay du facteur 3 |
| `ucp_agg_factor4_ratio_num` / `ucp_agg_factor4_ratio_den` | 3 / 2 | 1-100k | Ratio du facteur 4 |
| `ucp_agg_safety_qdelay_us` | 4000 | 0-100k | µs | Garde de sécurité 1 qdelay |
| `ucp_agg_safety_bdp_mult` | 3 | 1-100 | Multiplicateur BDP de garde de sécurité |
| `ucp_agg_max_window_ms` | 100 | 1-10000 | ms | Fenêtre de plafond extra_acked |
| `ucp_agg_max_decay_pct` | 75 | 0-100 | % | Taux de déclin du chien de garde |
| `ucp_agg_window_rotation_rtts` | 5 | 1-65535 | RTTs | Période de rotation de fenêtre |
| `ucp_agg_factor_weight` | 256 | 1-1024 | Score par facteur |
| `ucp_agg_confidence_max` | 1024 | 256-65535 | Confiance maximale |

### Coefficients EWMA

| Paramètre | Par défaut | Plage | Description |
|-----------|------------|-------|-------------|
| `ucp_ewma_qdelay_num` / `ucp_ewma_qdelay_den` | 7 / 8 | 0-100 / 1-100k | Poids EWMA qdelay |
| `ucp_ewma_jitter_num` / `ucp_ewma_jitter_den` | 7 / 8 | 0-100 / 1-100k | Poids EWMA de gigue |

### Divers

| Paramètre | Par défaut | Plage | Description |
|-----------|------------|-------|-------------|
| `ucp_probe_bw_cycle_len` | 8 | 2-256 | Phases du cycle PROBE_BW (puissance de deux) |
| `ucp_probe_bw_cycle_rand` | 8 | 1-cycle_len | Décalage aléatoire de phase de cycle |
| `ucp_full_bw_thresh_num` / `ucp_full_bw_thresh_den` | 125 / 100 | 0-100k / 1-100k | Seuil de croissance de sortie STARTUP |
| `ucp_full_bw_cnt` | 3 | 1-3 | Rounds sans croissance pour sortir |
| `ucp_probe_rtt_mode_ms_num` / `ucp_probe_rtt_mode_ms_den` | 200 / 1 | 1-100k | Durée de séjour PROBE_RTT |
| `ucp_pacing_margin_num` / `ucp_pacing_margin_den` | 0 / 100 | 0-50 / 1-100k | Marge de pacing (0 = aucune) |
| `ucp_probe_cwnd_bonus` | 2 | 0-100 | segs | Bonus cwnd de phase 0 |
| `ucp_bw_rt_cycle_len` | 10 | 2-256 | rounds | Longueur de fenêtre glissante BW |
| `ucp_cwnd_min_target` | 4 | 1-1000 | segs | Cwnd min (PROBE_RTT) |
| `ucp_bdp_min_rtt_us` | 1 | 0-100k | µs | Plancher min_rtt du BDP |
| `ucp_edt_near_now_ns` | 1000 | 0-10 M | ns | Seuil EDT quasi-maintenant |
| `ucp_min_tso_rate` | 1 200 000 | 1-1 Md | bytes/s | Seuil de bas débit TSO |
| `ucp_min_tso_rate_div` | 8 | 1-256 | Diviseur de débit TSO (base adaptative) |
| `ucp_tso_max_segs` | 127 | 1-65535 | segs | Segments TSO max |
| `ucp_tso_headroom_mult` | 3 | 0-1000 | Multiplicateur de marge TSO |
| `ucp_sndbuf_expand_factor` | 3 | 2-100 | Facteur d'expansion du tampon d'envoi |
| `ucp_ack_epoch_max` | 0xFFFFF | 64K-2G | bytes | Plafond d'époque ACK |
| `ucp_extra_acked_max_ms_num` / `ucp_extra_acked_max_ms_den` | 150 / 1 | 0-100k / 1-100k | Fenêtre max d'agrégation ACK |
| `ucp_probe_rtt_long_rtt_us` | 20000 | 0-10 M | µs | Seuil de long RTT |
| `ucp_probe_rtt_long_interval_div` | 1 | 1-1000 | Diviseur d'intervalle de long RTT |
| `ucp_drain_skip_qdelay_us` | 1000 | 0-100k | µs | Seuil qdelay de saut de DRAIN |
| `ucp_alone_confirm_rounds` | 3 | 1-32 | rounds | Rounds avant d'activer le mode flux unique |
| ucp_alone_qdelay_thresh_us | 1000 | 0-100k | µs | Délai de file max pour détection de flux unique |
| ucp_alone_jitter_thresh_us | 2000 | 0-100k | µs | Gigue max pour détection de flux unique |
| ucp_alone_agg_state_level | 1 | 0-2 | — | Rigueur d'agrégation (0=IDLE seulement, 1=≤SUSPECTED défaut, 2=≤CONFIRMED trop agressif) |
| `ucp_alone_bypass_ecn` | 1 | 0-1 | — | Ignorer le backoff ECN en mode solo (1=ignorer, 0=actif) |
| `ucp_alone_bypass_lt_bw` | 1 | 0-1 | — | Ignorer la condition LT BW en mode solo (1=ignorer, 0=actif) |

## Chemin de Données

```
ACK Arrive (rate_sample)
    │
    ▼
ucp_main()
    │
    ├──► Pipeline de confiance d'agrégation ACK (quand ucp_agg_enable)
    │      mesurer → évaluer → état → chien de garde
    │      ├── Couche de signal : mise à l'échelle R de Kalman (toujours active)
    │      └── Couche de contrôle : compensation cwnd (CONFIRMÉ+)
    │
    ├──► ucp_update_model()
    │      ├── ucp_update_bw()              BW max à fenêtre glissante
    │      ├── ucp_update_ecn_ewma()        Ratio de marque ECN-CE
    │      ├── ucp_update_ack_aggregation()  extra_acked à double fenêtre
    │      ├── ucp_update_cycle_phase()     Avancement de phase PROBE_BW
    │      ├── ucp_check_full_bw_reached()  Détection de sortie STARTUP
    │      ├── ucp_check_drain()            Entrée/sortie DRAIN + saut de DRAIN
    │      ├── ucp_update_min_rtt()         Kalman + fenêtre min-RTT + PROBE_RTT
    │      └── Attribution de gain spécifique au mode
    │
    ├──► ucp_apply_cwnd_constraints()
    │      └── ucp_ecn_backoff()            Recul ECN (cwnd_gain seulement)
    │
    ├──► ucp_set_pacing_rate()              immédiat, règle BBR
    │
    └──► ucp_set_cwnd()                    BDP + bornes + compensation agg
```

## Flux Interne du Filtre de Kalman

```
Échantillon RTT (rtt_us)
    │
    ├── Invalide (≥0 et < dynamic_max) ? Oui → rejeter
    │
    ├── Démarrage à froid (sample_cnt==0) ? Oui → init : x_est=z, p_est=max(p_init, rtt_us/div)
    │                                                     (contourne la porte max RTT)
    │
    ├── Q adaptatif : Q_base × max(q_min_factor, min_rtt_us / q_rtt_div)
    │   R adaptatif : R_base + max(0, jitter − jr_thresh) × R_base / jr_scale
    │
    ├── Innovation : innov = z − x_est
    │
    ├── Q-Boost: |innov| > boost_thresh && p_est ≤ converged && cooldown expired?
    │   ├── Yes: p_est = p_est_init, cooldown = 15, mark qboost_fired
    │   └── No:  cooldown-- if active
    │
    ├── Prédire : p_pred = p_est + Q
    │
    ├── Porte de valeurs aberrantes : |innov| > dyn_thresh && p_pred ≤ converged ?
    │   ├── Oui & reject_cnt < max → rejeter, ++consec_reject_cnt, retourner
    │   └── Oui & reject_cnt ≥ max → forcer l'acceptation (anti-verrouillage)
    │
    └── Mise à jour de Kalman :
         ├── K = p_pred / (p_pred + R)
         ├── x_est += K × innov (clampé non négatif)
         ├── p_est = max(p_floor, (1 − K) × p_pred)
         ├── Mise à jour EWMA de gigue
         ├── Mise à jour EWMA de qdelay
         ├── Estimation de bruit par covariance appariée BBR-S
         └── sample_cnt++
```

## Diagnostics

Interface de diagnostic compatible BBR via `ss -i` (`INET_DIAG_BBRINFO`) :

```
bbr_bw_lo/bbr_bw_hi : estimation de bande passante 64 bits (bytes/s)
bbr_min_rtt :         min_rtt_us actuel
bbr_pacing_gain :     gain de pacing actuel (BBR_UNIT, 256=1,0x)
bbr_cwnd_gain :       gain de cwnd actuel (BBR_UNIT)
```

## Performance Summary

Test environment: China → US LAX, 212ms RTT, 8 parallel flows, 26% packet loss, 1 Gbps shared VPS bottleneck.

| Metric | UCP v1.0 | BBR (control) | Delta |
|--------|----------|---------------|-------|
| Average throughput | 1,010 Mbps | 937 Mbps | **+7.8%** |
| Intra-UCP unfairness | 3.1× | 6.2× (BBR) | **−50%** |
| Worst single flow | 60.6 Mbps | 30.8 Mbps | **+97%** |
| Retransmits | 150K/10s | 137K/10s | +9.5% |
| Round-3 stability | 959 Mbps | 883 Mbps | **+8.6%** |

Retransmits are slightly higher — a trade-off consistent with maintaining high link utilisation under loss. UCP's Kalman-augmented min_rtt estimation provides a more accurate BDP baseline, allowing the algorithm to sustain higher throughput than BBRv1 on the same path.

---

*UCP v1.0 — construit sur BBRv1 (Cardwell et al. 2016, ACM Queue) et le filtre de Kalman (Kalman 1960).*

## Références

| Tag | Citation / Link |
|-----|----------------|
| BBR | Cardwell et al., "BBR: Congestion-Based Congestion Control", ACM Queue, Vol. 14 No. 5, 2016 — https://dl.acm.org/doi/10.1145/3009824 |
| BBR-S | "BBR-S: A Low-Latency BBR Modification for Fast-Varying Connections", 2021 — https://ieeexplore.ieee.org/document/9438951 |
| RBBR | "RBBR: A Receiver-Driven BBR in QUIC for Low-Latency in Cellular Networks", 2022 — https://ieeexplore.ieee.org/document/9703289 |
| ERCC | "ERCC: Fine-grained RDMA Congestion Control via Kalman Filter-based Multi-bit ECN Feedback Reconstruction", 2025 — https://dl.acm.org/doi/10.1145/3769270.3770124 |
| Linux BBR | Linux kernel BBR reference — https://github.com/torvalds/linux/blob/master/net/ipv4/tcp_bbr.c |
| Google BBR | BBR project page — https://github.com/google/bbr |
| BBRplus | "BBRplus: Adaptive Cycle Randomization, Drain-to-Target, and ACK Aggregation Compensation for BBR Convergence and Stall Prevention" — https://blog.csdn.net/dog250/article/details/80629551 |
| IETF 101 | "BBR Congestion Control Work at Google IETF 101 Update" — https://datatracker.ietf.org/meeting/101/materials/slides-101-iccrg-an-update-on-bbr-work-at-google-00 |
