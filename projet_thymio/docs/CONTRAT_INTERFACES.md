# Projet Thymio – Contrat d'interfaces (v0.1)

Les valeurs marquées **[À CONFIRMER]** sont à trancher en équipe. Les structures de données sont définies dans `interfaces.py` (source de vérité).

## 1. Flux de données

1. Vision → Filtre : `PoseMeasurement`
2. Contrôle → Filtre : `ControlInput`
3. Filtre → Planification et Contrôle : `PoseEstimate`
4. Vision → Planification : `Environment`
5. Planification → Contrôle : `Path`

## 2. Conventions globales

- **Repère monde** : origine = coin inférieur gauche du carré noir **[À CONFIRMER]** ; x vers la droite de l'image ; y vers le haut (conversion pixel → monde faite par Sujith) ; θ positif anti-horaire ; θ = 0 quand le Thymio regarde vers +x.
- **Unités** : position mm ; angle rad dans [−π, π] ; vitesse mm/s ; vitesse angulaire rad/s ; temps s.
- **Temps** : `time.monotonic()`. Chaque message porte l'instant de **capture** `t`. Pas du filtre : Ts = 0,1 s **[À CONFIRMER]**.
- Aucun pixel hors de Vision. Aucune unité Thymio brute (−500…500) hors de Contrôle.

## 3. Contrat module par module

**Vision → Filtre** : un `PoseMeasurement` par image, y compris quand le QR est caché (`visible=False`, champs à `None`). Coordonnées déjà en mm dans le repère monde. Sujith fournit l'écart-type du bruit de mesure (x, y, θ, robot immobile, ~200 images).

**Contrôle → Filtre** : un `ControlInput` par pas, avec la vitesse des roues **mesurée** par le Thymio (pas la consigne : pendant un évitement c'est ce que le robot fait vraiment). `v = (v_g + v_d)/2`, `omega = (v_d − v_g)/L`. Le contrôle fait des trajectoires courbes (v et omega en même temps), OK pour le filtre (Matéo), à confirmer avec Robin.

**Filtre → Planification, Contrôle** : un `PoseEstimate` à chaque pas Ts, avec ou sans mesure caméra. Si `kidnapped=True` : Robin replanifie, Matthieu arrête le robot le temps de la replanification.

**Vision → Planification** : `Environment` publié à l'initialisation puis si les obstacles changent. Obstacles en mm sans marge ; Robin applique l'inflation. Les limites noires sont traitées comme des obstacles.

**Planification → Contrôle** : `Path.waypoints` en mm. Taille de cellule choisie avec Matthieu. Robin supprime les waypoints alignés.

## 4. Cas limites

| Situation | Comportement |
|---|---|
| QR caché | `visible=False` ; le filtre prédit seulement, P grandit |
| QR retrouvé | le filtre corrige avec la mesure |
| Pas de mesure > 5 s **[À CONFIRMER]** | P dépasse un seuil, P4 arrête le robot |
| Kidnapping | `kidnapped=True`, réinitialisation sur la caméra, P3 replanifie |
| Aucun chemin | `Path.valid=False`, P4 ne bouge pas |
| Obstacle imprévu (IR) | évitement local par P4 ; le `ControlInput` reflète le mouvement réel |

## 5. Journal de calibration (à remplir)

| Paramètre | Valeur | Méthode | Date | Responsable |
|---|---|---|---|---|
| Facteur Thymio → mm/s | 0,337 mm/s par unité (0,353 à 35 mm/s) | ligne droite à 227 unités (≈ 80 mm/s) mesurée à la règle, vérifié avec la boucle principale : 390 mm à la règle, 388 estimés | 19/11/2026 | Matéo + Matthieu |
| Empattement L (mm) | 96,5 | rotation sur place à ±227 unités pendant 8 s, angle réel mesuré sur photo (720° ± 2°) | 19/11/2026 | Matthieu |
| Bruit caméra σx, σy (mm) | | | | Sujith |
| Bruit caméra σθ (rad) | | | | Sujith |
| σ_v, σ_ω (bruit commande) | σ_v ≈ 3,6 mm/s, σ_ω ≈ 0,075 rad/s | écart-type des vitesses **mesurées** (motor.speed) à 227 unités : ≈ 5 mm/s par roue en ligne droite et en rotation, puis σ_v = σ_roue/√2, σ_ω = √2·σ_roue/L | 19/11/2026 | Matéo (mesures Matthieu) |
| Q plancher | | | | Matéo |
| Taille de cellule (mm) | | | | Robin + Matthieu |
| Ts (s) | 0,1 | boucle mesurée à 101 à 105 ms par pas sur le vrai robot, mesures du Thymio reçues toutes les 50 ms | 19/11/2026 | tous |

## 6. À confirmer

1. Origine du repère monde.
2. Ts = 0,1 s adapté à la caméra et à la communication Thymio ?
3. Orientation du QR code sur le robot et décalage de θ.
4. Taille de cellule de la grille.
5. Seuil de perte de mesure avant arrêt du robot.
