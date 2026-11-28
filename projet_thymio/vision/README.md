# Module Vision

Ce module fournit au reste du projet les informations mesurees par la camera :

- pose du Thymio ;
- obstacles presents sur le terrain ;
- limites de la zone utile ;
- position du but represente par un cercle rouge plein.

Toutes les valeurs sorties sont exprimees en millimetres ou en radians. Aucun calcul ou message dependant d'un autre module interne n'est utilise hors des structures definies dans `interfaces.py`.

## Fichiers

| Fichier | Role |
|---|---|
| `detect_arena.py` | Detecte les quatre coins du carre noir delimitant le terrain |
| `perspective.py` | Calcule la matrice de perspective et transforme l'image en vue de dessus |
| `detect_robot.py` | Detecte le marqueur ArUco du Thymio et calcule sa pose |
| `detect_obstacles.py` | Detecte les obstacles sombres et les transforme en polygones |
| `detect_goal.py` | Detecte le cercle rouge plein representant le but |
| `realtime.py` | Assemble la webcam, les fonctions de vision et les interfaces du projet |
| `calibration.py` | Centralise les dimensions du terrain et les conversions pixel/mm |

## Flux de donnees

```text
Webcam
  -> detection du terrain
  -> correction de perspective
  -> detection du robot
  -> PoseMeasurement vers le filtre

Webcam corrigée
  -> detection des obstacles
  -> detection du cercle rouge
  -> Environment vers le planificateur
```

## Structures utilisees

### PoseMeasurement

`detect_robot()` retourne une `PoseMeasurement` pour chaque image :

- `visible=True` : le marqueur a ete detecte ;
- `visible=False` : aucun marqueur n'a ete detecte ;
- `x` et `y` : position du centre du marqueur en mm ;
- `theta` : orientation du robot en radians ;
- `t` : instant de capture de l'image.

La position du marqueur doit etre alignee avec le milieu entre les deux roues.

### Environment

`RealCamera.environment()` retourne un `Environment` contenant :

- les dimensions utiles du terrain ;
- les polygones des obstacles en mm ;
- la position du but detecte.

## Mode camera sans Thymio

Depuis la racine du projet :

```bash
python -m controle.main --camera --goal 400 100
```

Ce mode permet de tester :

- la calibration du terrain ;
- la detection du robot ;
- la detection des obstacles ;
- la detection du cercle rouge ;
- le filtre Kalman ;
- le calcul du chemin ;
- l'affichage du resultat.

Aucune commande moteur n'est envoyée au Thymio.

## Mode reel

Apres avoir configure la communication avec le Thymio :

```bash
python -m controle.main --reel --goal 400 100
```

Ce mode utilise la vraie webcam, le filtre Kalman, le planificateur et les commandes moteur.

## Tests autonomes

Chaque fichier peut etre teste independamment :

```bash
python -m vision.detect_arena
python -m vision.detect_obstacles
python -m vision.detect_robot
python -m vision.perspective
python -m vision.detect_goal
```

Appuyer sur `q` pour fermer une fenetre.

## Parametres importants

Dans `detect_arena.py` :

- filtre gaussien ;
- seuillage Otsu ;
- precision de l'approximation polygonale.

Dans `perspective.py` :

- dimensions de l'image vue de dessus ;
- marge autour du terrain.

Dans `detect_obstacles.py` :

- seuil fixe des pixels sombres ;
- aire minimale des obstacles ;
- masque applique sur le marqueur du Thymio.

Dans `detect_robot.py` :

- dictionnaire ArUco `DICT_4X4_50` ;
- identifiant du marqueur `0` ;
- conversion pixels vers millimetres ;
- calcul de theta.
- seuils adaptatifs du detecteur ;
- raffinement subpixel des coins ;

Dans `detect_goal.py` :

- plages de couleur rouge en HSV ;
- aire minimale du cercle ;
- seuil de circularite.

Dans `calibration.py` :

- largeur et hauteur du terrain en mm ;
- taille de l'image vue de dessus ;
- marge autour du terrain ;
- fonctions communes de conversion pixel/mm.

## Calibration du bruit camera

Lancer le programme avec le Thymio immobile et son marqueur bien visible :

```bash
python -m vision.detect_robot
```

Le programme conserve les valeurs uniquement lorsque `visible=True` jusqu'à obtenir 200 mesures.
Il affiche ensuite les mesures `x`, `y`, `theta`, les trois ecarts-types et une ligne directement utilisable dans `filtre/params.py`.

Valeurs actuellement mesurees :

```python
SIGMA_CAM = np.array([0.264850, 1.271449, 0.005302])
R = np.diag(SIGMA_CAM ** 2)
```

Ces valeurs sont exprimees en :

- mm pour `x` ;
- mm pour `y` ;
- radian pour `theta`.

## Changements récents (28/11/2026)

- Les coordonnées `x`, `y` et l'angle `theta` ne sont plus écrits en surimpression sur l'image brute de la caméra; les informations restent dans le panneau d'état et la modélisation 2D.
- L'affichage distingue l'état caméra de l'état du filtre : le statut « PREDICTING » dépend d'images manquées consécutives (seuil réglé à 10 dans le contrôle).
- Pendant la prédiction, la modélisation 2D affiche une ellipse de confiance à 95 %, calculée et orientée avec la covariance positionnelle x/y du filtre.
