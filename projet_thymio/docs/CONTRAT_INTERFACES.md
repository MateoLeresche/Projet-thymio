# Projet Thymio – Contrat d'interfaces (v0.1)

Les valeurs marquées **[À CONFIRMER]** sont à trancher en équipe. Les structures de données sont définies dans `interfaces.py` (source de vérité).

## 1. Flux de données

1. Vision → Filtre : `PoseMeasurement`
2. Contrôle → Filtre : `ControlInput`
3. Filtre → Planification et Contrôle : `PoseEstimate`
4. Vision → Planification : `Environment`
5. Planification → Contrôle : `Path`

## 2. Conventions globales

- **Repère monde** : origine = coin inférieur gauche du carré noir **[À CONFIRMER]** ; x vers la droite de l'image ; y vers le haut (conversion pixel → monde faite par P1) ; θ positif anti-horaire ; θ = 0 quand le Thymio regarde vers +x.
- **Unités** : position mm ; angle rad dans [−π, π] ; vitesse mm/s ; vitesse angulaire rad/s ; temps s.
- **Temps** : `time.monotonic()`. Chaque message porte l'instant de **capture** `t`. Pas du filtre : Ts = 0,1 s **[À CONFIRMER]**.
- Aucun pixel hors de Vision. Aucune unité Thymio brute (−500…500) hors de Contrôle.

## 3. Contrat module par module

**Vision → Filtre** : un `PoseMeasurement` par image, y compris quand le QR est caché (`visible=False`, champs à `None`). Coordonnées déjà en mm dans le repère monde. P1 fournit l'écart-type du bruit de mesure (x, y, θ, robot immobile, ~200 images).

**Contrôle → Filtre** : un `ControlInput` par pas, avec la vitesse réellement commandée. `v = (v_g + v_d)/2`, `omega = (v_d − v_g)/L`. Le robot fait soit une translation pure (omega = 0), soit une rotation sur place (v = 0) : pas de trajectoire courbe.

**Filtre → Planification, Contrôle** : un `PoseEstimate` à chaque pas Ts, avec ou sans mesure caméra. Si `kidnapped=True` : P3 replanifie, P4 arrête le robot le temps de la replanification.

**Vision → Planification** : `Environment` publié à l'initialisation puis si les obstacles changent. Obstacles en mm sans marge ; P3 applique l'inflation. Les limites noires sont traitées comme des obstacles.

**Planification → Contrôle** : `Path.waypoints` en mm. Taille de cellule choisie avec P4 (départ : 20 mm **[À CONFIRMER]**). P3 supprime les waypoints alignés.

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
| Facteur Thymio → mm/s | | | | P2 + P4 |
| Empattement L (mm) | | | | P4 |
| Bruit caméra σx, σy (mm) | | | | P1 |
| Bruit caméra σθ (rad) | | | | P1 |
| σ_v, σ_ω (bruit commande) | | | | P2 |
| Q plancher | | | | P2 |
| Taille de cellule (mm) | | | | P3 + P4 |
| Ts (s) | | | | tous |

## 6. À confirmer

1. Origine du repère monde.
2. Ts = 0,1 s adapté à la caméra et à la communication Thymio ?
3. Orientation du QR code sur le robot et décalage de θ.
4. Taille de cellule de la grille.
5. Seuil de perte de mesure avant arrêt du robot.
