# Module contrôle (Matthieu)

Commande du Thymio (tdmclient), suivi du chemin, évitement local des obstacles imprévus.

## Fichiers

| Fichier | Rôle |
|---|---|
| `main.py` | **Boucle principale du projet** : caméra → filtre (Matéo) → chemin (Robin) → suivi + évitement → moteurs. Gère le kidnapping (capteurs de sol), le QR caché, le replanning après un évitement. |
| `fake_modules.py` | Fausse caméra et faux planificateur (mêmes structures que interfaces.py), en attendant les vrais modules. |
| `messages.py` | **Seul fichier qui lit ou fabrique les structures de `interfaces.py`** : `PoseEstimate` → pose, `Path` → points, vitesses des roues → `ControlInput`, arrêt si kidnapping ou filtre trop incertain. |
| `motion_model.py` | Valeurs mesurées du robot, conversion unités Thymio ↔ mm/s, cinématique, odométrie. |
| `thymio.py` | Seul fichier qui parle au robot (tdmclient) : lire capteurs et vitesses, commander les moteurs et les LEDs. |
| `controller.py` | Suivi du chemin, loi fluide du cours (Astolfi adapté : rotation ∝ α, avance ∝ cos α, ralentissement à l'arrivée). |
| `local_avoidance.py` | Évitement des obstacles vus par les capteurs infrarouges (Braitenberg). |
| `calibration.py` | Mesures sur le vrai robot pour remplir les valeurs MESURER. |
| `simulator.py` | Faux Thymio, pour tester sans robot. |
| `demo_mouvement.py` | Suit un carré de 40 cm avec évitement, position par odométrie seule. Modèle de la boucle principale. |
| `comparaison_controleurs.svg` | Ancienne loi (pivoter puis avancer) contre loi fluide, en simulation. Pour le rapport. |

Tests : `tests/test_controle.py`.

## Lancer (depuis le dossier `projet_thymio/`)

```bash
python -m controle.main --sim --plot data/main_sim.png # boucle complète simulée : obstacle, QR caché, kidnapping
python -m controle.main --reel --goal 400 0            # vrai robot, sans caméra (le filtre ne fait que prédire)
python -m controle.demo_mouvement --sim                # faux robot, un obstacle sur le premier côté
python -m controle.demo_mouvement --sim --no-obstacle
python -m controle.demo_mouvement --sim --plot data/trajectoire.png
python -m controle.demo_mouvement                      # vrai Thymio, posé au départ du carré, regardant vers +x
python -m controle.calibration vitesse                 # puis rotation, prox, sol
pytest
```

Résultat actuel en simulation : carré bouclé, obstacle contourné sans le toucher, fin à environ 1 cm du but. Rien n'est encore testé sur le vrai Thymio.

## Brancher les vrais modules dans main.py

Dans `main()`, deux lignes à changer quand les modules sont prêts :

| Ligne actuelle | À remplacer par | Ce que la fonction doit faire |
|---|---|---|
| `vision = FakeCamera(...)` | la caméra de Sujith | `vision(t)` renvoie un `PoseMeasurement` : pose du **milieu de l'axe des roues** en mm et rad dans le repère monde, `t` = instant de capture, `visible=False` si le QR est caché |
| `plan = straight_line_plan` | le planificateur de Robin | `plan(start, goal)` renvoie un `Path` : seulement les coins, marge comprise, `valid=False` si pas de chemin ; appelable depuis n'importe quelle position |

Le filtre (`filtre.filter_node.FilterNode`) est déjà le vrai. Après un kidnapping, `main.py` crée un nouveau `FilterNode`, qui se réinitialise sur la prochaine mesure caméra visible.

## Échanges avec les autres modules

| Avec | Structure | Utilisation ici |
|---|---|---|
| Filtre (Matéo) → moi | `PoseEstimate` | `messages.pose_tuple(est)` pour suivre le chemin ; `messages.must_stop(est)` arrête le robot si `kidnapped` ou si l'écart-type de position dépasse `POSITION_STD_MAX_MM` (50 mm, à confirmer). |
| Moi → filtre | `ControlInput` | `messages.control_input(t, v_left, v_right)` avec les vitesses **mesurées** (`Thymio().read()`), pas les consignes : pendant un évitement, c'est ce que le robot fait vraiment. |
| Planification (Robin) → moi | `Path` | `messages.path_points(path)` ; `None` si `valid=False` : le robot ne bouge pas. Puis `follower.set_path(points)`. |

**Pour Robin** : seulement les coins du chemin (pas chaque case de la grille, sinon le robot zigzague). Dans les virages, le robot coupe d'environ 1 à 2 cm : à compter dans la marge. Il faut une fonction qu'on peut rappeler depuis n'importe quelle position, pour replanifier après un évitement ou un kidnapping.

**Pour Matéo** : la loi fluide fait des trajectoires courbes (v et ω non nuls en même temps), alors que le contrat (section 3) dit « translation pure ou rotation sur place ». Ta prédiction `x += v·cos θ·dt, y += v·sin θ·dt, θ += ω·dt` marche aussi pour les courbes. Simulation (20 essais, carré de 40 cm, caméra bruitée à 2 mm, QR caché 4 s) : erreur moyenne du filtre 1,0 mm, erreur maximale pendant que le QR est caché 5,5 mm, contre 1,0 et 4,8 mm avec l'ancienne loi. Si on garde quand même la règle du contrat : `CONTROL_LAW = "turn_then_go"` dans `controller.py`.

**Pour Sujith** : `thymio.is_lifted(lecture)` dit si les capteurs de sol ne voient plus le sol (robot soulevé), pour aider à détecter le kidnapping.

## Valeurs à mesurer (MESURER dans le code)

À reporter aussi dans le journal de calibration de `docs/CONTRAT_INTERFACES.md` (méthode et date).

| Valeur | Fichier | Mesure |
|---|---|---|
| `SPEED_FACTOR` (facteur Thymio → mm/s) | motion_model.py | `calibration vitesse` |
| `LEFT_CORRECTION` | motion_model.py | `calibration vitesse` (déviation en ligne droite) |
| `WHEEL_SPEED_STD_MMS` | motion_model.py | `calibration vitesse` (sert à Matéo pour σ_v, σ_ω) |
| `WHEEL_BASE_MM` (empattement L) | motion_model.py | `calibration rotation` |
| `PROX_ENTER`, `PROX_EXIT` | local_avoidance.py | `calibration prox` |
| `GROUND_LIFTED_THRESHOLD` | thymio.py | `calibration sol` |
| `K_ALPHA`, `V_MAX`, `K_RHO` | controller.py | Essais sur le vrai robot (noter les trajectoires pour le rapport) |

## Règles pour la démo

- Les obstacles posés pendant la démo doivent être **clairs et hauts d'au moins 5 cm** : le noir absorbe l'infrarouge.
- La carte est faite **une seule fois au démarrage**, sinon la caméra ajoute l'obstacle posé à la carte et on ne voit plus l'évitement local.
- Ne pas poser un obstacle à moins d'environ 15 cm d'un point de passage : le robot ne peut pas l'atteindre sans le toucher.
