# Module filtre (Matéo)

Estime la pose du Thymio (x, y, theta) avec un filtre de Kalman étendu (EKF). Il fusionne la vitesse mesurée des roues (prédiction) et la position donnée par la caméra (correction). Si le QR code est caché, il continue avec la prédiction seule et l'incertitude `P` grandit.

Les autres modules n'ont besoin de connaître que **`FilterNode`** et les structures de `interfaces.py`.

## Les fichiers

| Fichier | À quoi il sert |
|---|---|
| `filter_node.py` | **Le seul fichier à utiliser.** Reçoit `ControlInput` et `PoseMeasurement`, renvoie `PoseEstimate`. Gère aussi le rejet des mesures aberrantes, la détection de kidnapping et la compensation du retard caméra. |
| `kalman.py` | Les maths du filtre (prédiction, correction, innovation). Ne connaît ni la caméra, ni tdmclient, ni `interfaces.py`. |
| `angles.py` | `wrap(a)` : ramène un angle dans [-π, π]. |
| `params.py` | Les paramètres du filtre (bruit caméra, bruit de commande, Q, seuils). |

Il faut le dossier `filtre/` complet (les fichiers s'importent entre eux), `interfaces.py` à la racine, et `numpy`. Lancer depuis le dossier `projet_thymio/`.

## Utilisation

```python
import time
from interfaces import PoseMeasurement, ControlInput
from filtre.filter_node import FilterNode

kalman = FilterNode()          # paramètres de params.py, EKF + compensation du retard

# à chaque pas (Ts = 0,1 s) :
kalman.on_control(ControlInput(t=now, v=v_mesuree, omega=omega_mesuree))
kalman.on_measurement(PoseMeasurement(t=t_image, visible=True, x=..., y=..., theta=...))
estimate = kalman.step(time.monotonic())    # PoseEstimate, ou None

if estimate is not None:
    x, y, theta, P = estimate.x, estimate.y, estimate.theta, estimate.P
    if estimate.kidnapped:
        ...                                  # replanifier, arrêter le robot le temps du calcul
```

Ordre à chaque `step` : prédiction avec la dernière commande reçue, puis correction si une mesure visible est arrivée.

## Les règles à respecter

1. **Appeler `step()` à chaque pas**, avec ou sans mesure caméra. Sans mesure : prédiction seule, `P` grandit, `measurement_used = False`.
2. **`step()` renvoie `None`** tant que le filtre n'a pas reçu une première mesure avec `visible=True`. Dans ce cas le robot doit rester à l'arrêt.
3. **Donner chaque image une seule fois** à `on_measurement`. Si on redonne la même image, le filtre la compte plusieurs fois et se croit trop sûr de lui. Comparer le `t` de l'image avec celui de la précédente (c'est ce que fait `controle/main.py`).
4. **`ControlInput` = vitesse mesurée des roues**, pas la consigne : `v = (v_g + v_d) / 2`, `omega = (v_d − v_g) / L`. Côté contrôle c'est déjà fait par `messages.control_input`.
5. **Unités** : mm, rad dans [-π, π], mm/s, rad/s. `t` vient de `time.monotonic()` et représente l'instant de **capture** de l'image (pas du traitement), sinon la compensation du retard ne marche pas.
6. Le filtre n'importe ni OpenCV ni tdmclient, il ne faut pas en ajouter.

## Le modèle

État `x = [x, y, theta]`, commande `u = [v, omega]` :

```
x_{k+1} = x_k + B(theta_k) u_k + w_k        B = [[cos(th)·dt, 0], [sin(th)·dt, 0], [0, dt]]
z_k     = x_k + v_k                         (la caméra mesure directement x, y, theta)
```

- **EKF** : le robot avance et tourne en même temps (loi fluide du contrôle), donc la position dépend de `theta` de façon non linéaire. À la prédiction, `P = F P Fᵀ + Q` avec la Jacobienne
  `F = [[1, 0, -v·dt·sin(th)], [0, 1, v·dt·cos(th)], [0, 0, 1]]`.
  C'est ce qui permet de ne pas sous-estimer l'incertitude sur la position quand le QR est caché pendant un virage. `KalmanFilter(use_jacobian=False)` redonne le KF simple (`A = I`), utile pour la comparaison.
- `Q = B · diag(σ_v², σ_ω²) · Bᵀ + Q_floor` : le bruit sur la commande propagé dans l'état, plus un petit bruit de modèle (glissement, jeu mécanique).
- L'innovation sur l'angle est ramenée dans [-π, π] avec `wrap` (sinon un écart de 178° → -178° donnerait 356°).
- La correction utilise la forme de Joseph, ce qui garde `P` symétrique et définie positive.

## Mesures aberrantes, kidnapping, retard

- **Rejet des aberrations** : à chaque mesure on calcule le NIS (innovation normalisée par `S`). Au-dessus de `NIS_GATE` (11,34, χ² à 3 ddl, 99 %) la mesure n'est pas utilisée.
- **Kidnapping** : après `N_KIDNAP = 20` rejets consécutifs (environ 2 s à 10 Hz), le filtre se réinitialise sur la mesure caméra et publie `kidnapped = True` pendant ce pas. Ce délai réduit les faux déclenchements dus à quelques mesures bruitées. Un QR caché, lui, n'est **pas** un kidnapping (pas de mesure, donc pas de rejet). Ça ne marche que si la caméra voit le robot après avoir été déplacé. Côté contrôle, le robot soulevé est aussi détecté avec les capteurs de sol (`thymio.is_lifted`).
- **Retard caméra** : le filtre garde 2 s d'historique (états, covariances, commandes). Une mesure datée de `t` est appliquée à l'état de cet instant, puis les prédictions suivantes sont rejouées jusqu'à maintenant. Efficace jusqu'à environ 0,2 s de retard, au-delà la précision se dégrade (voir `compare_latence.py`). Désactivable avec `FilterNode(compensate=False)`.

## Paramètres (`params.py`)

| Paramètre | Rôle | Origine |
|---|---|---|
| `TS` | pas de temps (0,1 s) | mesuré sur le vrai robot : 101 à 105 ms par pas |
| `SIGMA_CAM` | écart-type caméra (x, y, theta) → `R` | **PLACEHOLDER, à fournir par Sujith** (robot immobile, ~200 images) |
| `SIGMA_V`, `SIGMA_W` | bruit sur la commande | mesuré : 3,6 mm/s et 0,075 rad/s (vitesse mesurée à 227 unités, voir journal de calibration) |
| `Q_FLOOR` | bruit de modèle non expliqué | **PLACEHOLDER, à régler** |
| `NIS_GATE`, `N_KIDNAP` | rejet de mesure et détection de kidnapping | χ² 99 % à 3 ddl ; 20 rejets de suite |

Tous les paramètres peuvent être surchargés pour un test :

```python
FilterNode(R=..., sigma_v=..., sigma_w=..., q_floor=..., use_jacobian=True, compensate=True)
```

Chaque valeur modifiée doit être notée avec sa méthode et sa date dans `docs/CONTRAT_INTERFACES.md` (journal de calibration).

## Validation (simulation, la vérité terrain est connue)

```bash
pytest tests/test_kalman.py tests/test_simulation.py
python -m evaluation.run_validation       # tableau Monte Carlo + figures dans docs/figures/
python -m evaluation.compare_kf_ekf       # KF contre EKF (carré, cercle, QR caché, odométrie seule)
python -m evaluation.compare_latence      # effet du retard caméra, avec et sans compensation
```

Un filtre est **cohérent** si, sur de nombreux essais :

- NEES moyen ≈ 3 (erreur réelle compatible avec `P`),
- NIS moyen ≈ 3 (innovations compatibles avec `S`),
- couverture ±3σ ≈ 99,7 %.

Cas testés : caméra à 10 Hz et 2 Hz, QR caché 2 s et 5 s, odométrie seule, `Q` ou `R` trop petits (NEES ≫ 3), kidnapping, pas de fausse alarme en nominal, retard de 0,2 s avec et sans compensation.

## Limites connues

- `SIGMA_CAM` et `Q_FLOOR` sont des placeholders : tant qu'ils ne sont pas mesurés, les NEES/NIS de la simulation valent pour des paramètres choisis, pas pour la vraie caméra.
- Le simulateur suppose un bruit gaussien et des mesures caméra indépendantes ; la vraie caméra peut avoir des erreurs systématiques (homographie, décalage du QR).
- La détection de kidnapping dépend de la caméra : sans mesure visible après le déplacement, le filtre ne peut pas le voir.

## Pour chacun

**Sujith** : j'ai besoin de l'écart-type de x, y, theta mesuré robot immobile (~200 images) pour remplir `SIGMA_CAM`. Publier un `PoseMeasurement` à chaque image, avec `t` = instant de capture, et `visible=False` si le QR n'est pas détecté. La position doit être celle du milieu entre les deux roues.

**Robin** : replanifier quand `PoseEstimate.kidnapped` vaut `True`. `P` (covariance 3×3) est dans `PoseEstimate` si tu veux t'en servir.

**Matthieu** : `controle/main.py` respecte déjà les règles 1 à 4. Le filtre signale maintenant `kidnapped` lui-même, en plus de ton test avec les capteurs de sol.

## Changements récents (28/11/2026)

- Le signal de kidnapping par mesures caméra aberrantes demande maintenant 20 rejets consécutifs (environ 2 s à 10 Hz), afin d'éviter qu'un écart bref déclenche une fausse alarme.
- La prédiction en absence de mesure reste disponible dans `PoseEstimate`, avec sa covariance `P`; la commande peut s'arrêter selon le seuil de sécurité sans interrompre le calcul du filtre.
