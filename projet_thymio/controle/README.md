# Module contrôle (Matthieu)

Ici il y a tout ce qui fait bouger le Thymio : la communication avec le robot, le suivi du chemin, l'évitement des obstacles imprévus, et la boucle principale qui relie tous les modules.

Tout a été testé sur le vrai robot, sauf avec la caméra de Sujith (branchée dans `main.py` mais pas encore essayée avec le robot). Les mesures et les essais sont dans `mesures_reelles.md`.

## Les fichiers

| Fichier | À quoi il sert |
|---|---|
| `main.py` | La boucle principale du projet : caméra, filtre de Matéo, chemin de Robin, suivi + évitement, moteurs. Elle gère aussi le kidnapping, le QR caché et la reprise du chemin après un contournement. |
| `fake_modules.py` | Une fausse caméra, une fausse carte et un planificateur en ligne droite (quand on n'a pas de carte). Ils renvoient les mêmes structures que `interfaces.py`. |
| `messages.py` | Le seul fichier qui lit ou fabrique les structures de `interfaces.py` (PoseEstimate, Path, ControlInput). |
| `motion_model.py` | Les valeurs mesurées du robot, la conversion unités Thymio / mm/s, l'odométrie. |
| `thymio.py` | Le seul fichier qui parle au robot (tdmclient). |
| `controller.py` | Le suivi du chemin avec la loi fluide du cours (Astolfi adapté). |
| `local_avoidance.py` | Le contournement des obstacles vus par les capteurs infrarouges (Bug 2 du cours) : le robot longe l'obstacle et reprend le chemin dès qu'il retombe dessus. |
| `simulator.py` | Un faux Thymio pour tester sans le robot. |
| `demo_mouvement.py` | Le carré de 40 cm avec évitement, utilisé pour régler les virages et l'évitement. |
| `calibration.py` | Les mesures à faire sur le robot (vitesse, rotation, capteurs). |
| `mesures_reelles.md` | Le journal de tous les essais sur le vrai robot, avec les chiffres (pour le rapport). |

Les tests sont dans `tests/test_controle.py`.

## Lancer (depuis le dossier `projet_thymio/`)

```bash
python -m controle.main --sim --plot data/main_sim.png   # tout simulé : boîte sur la carte, obstacle imprévu, QR caché, kidnapping
python -m controle.main --camera                         # caméra + filtre + planificateur, sans Thymio
python -m controle.main --reel                           # vrai robot avec la caméra
python -m controle.main --odometrie --goal 400 0         # vrai robot sans caméra, 40 cm tout droit
python -m controle.demo_mouvement --sim                  # carré de 40 cm sur le faux robot
python -m controle.calibration vitesse                   # puis rotation, prox, sol
pytest
```

Il faut `tdmclient` pour le vrai robot (`pip install tdmclient`) et Thymio Suite ouvert, sans ouvrir de langage dedans (sinon il bloque le robot).
Après l'arrivée au but, ou pendant que le robot est soulevé, le robot reste arrêté tandis que le programme continue d'actualiser la caméra et l'interface. Après avoir reposé le robot, le filtre et le chemin sont réinitialisés et le déplacement reprend vers le but. Appuyer sur `q` dans l'interface ou sur `Ctrl+C` dans le terminal pour quitter.

## Les modules dans main.py

Tout est branché :

- **Caméra de Sujith** (`vision.realtime.RealCamera`) : `vision(t)` renvoie un `PoseMeasurement` (position du **milieu entre les deux roues** en mm et rad, `t` = moment où l'image a été **prise**), et `vision.environment(goal)` renvoie la carte une seule fois au début.
- **Planificateur de Robin** : `planner.set_environment(carte)`, puis `main.py` appelle `planner.plan(estimation, goal)` au départ et après un kidnapping. Après un obstacle imprévu on ne recalcule pas de chemin : le robot contourne l'obstacle et reprend le chemin de base quand il retombe dessus (Bug 2). On redemande un chemin seulement s'il a fait 1,5 m autour sans le retrouver.
- **Filtre de Matéo** : voir plus bas.

En simulation (`--sim`) la caméra et la carte sont remplacées par `FakeCamera` et `fake_map` de `fake_modules.py`, donc pas besoin d'OpenCV pour la simulation et les tests.

Sur le vrai robot sans caméra (`--odometrie`) on n'a pas de carte, donc on garde la ligne droite (`straight_line_plan`).

Le filtre (`filtre.filter_node.FilterNode`) est déjà le vrai. À chaque pas, `main.py` fait `on_control`, puis `on_measurement` **seulement si c'est une nouvelle image** (on compare le `t` de l'image avec la précédente, donc Sujith peut redonner la même image tant qu'il n'y en a pas de nouvelle), puis `step(time.monotonic())`. Tant que le filtre n'a pas vu le QR une première fois, `step` renvoie `None` et le robot reste arrêté. Le kidnapping est vu de deux façons : les capteurs de sol (robot soulevé, `thymio.is_lifted`) ou le filtre de Matéo (`kidnapped=True` quand la caméra ne colle plus à la prédiction 20 fois de suite). Dans les deux cas le robot s'arrête et on redemande un chemin. Quand il est reposé on crée un nouveau `FilterNode`, qui reprend la position de la prochaine image.

## Pour chacun

**Matéo** : je t'envoie la vitesse **mesurée** des roues dans `ControlInput` (c'est ce que le robot fait vraiment, même pendant un évitement). Le robot avance et tourne en même temps (loi fluide), ta prédiction marche aussi avec ça. Valeurs mesurées à 80 mm/s : facteur 0,337, empattement 96,5 mm, sigma_v environ 3,6 mm/s, sigma_w environ 0,075 rad/s. Les sigma viennent des vitesses **mesurées** (motor.speed), pas de la consigne, et donnent la même chose en ligne droite et en rotation (environ 5 mm/s par roue). Le facteur est bon aussi en rotation : 720° réels pour 2 tours demandés. Avec ça ton filtre se trompe de 2 mm sur 40 cm sans caméra. Tout est dans le journal de calibration de `docs/CONTRAT_INTERFACES.md`.

**Robin** : ton planificateur est branché dans `main.py` et marche en simulation (contourne la boîte de la carte après le kidnapping, sans collision). Marge de 80 mm : le robot fait 11 cm de large (5,5 cm de chaque côté), il coupe les virages d'environ 1,5 cm, et à 60 mm les capteurs voyaient l'obstacle connu et lançaient l'évitement local pour rien.

**Sujith** : la position doit être celle du milieu entre les roues (le trou au centre du robot, là où on met un feutre), pas le centre du QR. `thymio.is_lifted(lecture)` dit si le robot est soulevé, ça peut t'aider pour le kidnapping.

## Valeurs mesurées (détails dans mesures_reelles.md)

| Valeur | Où | Combien | Comment |
|---|---|---|---|
| Facteur Thymio vers mm/s | motion_model.py | 0,337 | ligne droite à 80 mm/s |
| Empattement | motion_model.py | 96,5 mm | 2 tours sur place, photo |
| Gain de rotation K_ALPHA | controller.py | 3 | carrés avec 1,5, 2 et 3 |
| Seuil d'évitement PROX_ENTER | local_avoidance.py | 1000 | l'obstacle de la démo ne dépasse jamais 2300 (1239 à 6 cm, 0 à 8 cm) |
| Contournement K_FOLLOW, R_ARC | local_avoidance.py | 0,001 et 16 cm | simulation, puis 4 essais réels sans contact (passe à environ 5 cm) |
| Seuil robot soulevé | thymio.py | 100 | capteurs de sol posé et soulevé |

## Pour la démo

- Les obstacles ajoutés doivent être **clairs et hauts d'au moins 5 cm** (le noir renvoie mal l'infrarouge).
- La carte est faite **une seule fois au début**, sinon la caméra ajoute l'obstacle à la carte et on ne voit plus l'évitement.
- Pas d'obstacle imprévu à moins de 15 cm d'un coin du chemin.
- Le but doit être à au moins 8 cm du cadre noir et des obstacles de la carte, sinon le planificateur ne trouve pas de chemin (`valid=False`) et le robot ne bouge pas.
- Débrancher le câble de charge pendant les essais, il fausse la vitesse mesurée.
- À la fin de `main.py --reel`, la ligne « durée d'un pas » doit être proche de 100 ms. En simulation, avec des pas de 200 ms le robot touche déjà le gobelet : il voit l'obstacle trop tard.
- Avec un autre Thymio ou un autre obstacle, refaire `python -m controle.calibration prox` avec l'obstacle de la démo à 4, 6 et 8 cm : les capteurs ne donnent pas tous la même valeur. Avec la boîte blanche on montait à 4300, avec l'obstacle de la démo jamais plus de 2300, d'où `PROX_ENTER = 1000`. On peut tester un autre seuil sans toucher au code : `python -m controle.main --reel --prox-enter 1200`. À la fin, main.py affiche la plus grande valeur vue par les capteurs avant.

## Changements récents (28/11/2026)

- À l'arrivée au but, les moteurs restent arrêtés mais la boucle continue; `q` dans l'interface ou `Ctrl+C` dans le terminal permet de quitter.
- La capture caméra continue pendant que le robot est soulevé. Après repose, le filtre et le chemin sont réinitialisés, puis le robot reprend vers le but.
- Le compteur de perte caméra compte les nouvelles images : le mode « PREDICTING » s'affiche après 10 images manquées et revient à l'état caméra après 3 détections consécutives.
- Le filtre confirme un kidnapping après 20 mesures aberrantes consécutives.
