# Mesures sur le vrai Thymio (journal brut, pour le rapport)

Date : 2026-11-19. Robot sur une table, relié en USB, en charge.

## Vitesse mesurée à l'arrêt (robot immobile, consigne 0)

| Lecture | Gauche (mm/s) | Droite (mm/s) | Remarque |
|---|---|---|---|
| 1 | -5.6 | 0.0 | |
| 2 | -17.2 | -14.4 | |
| 3 | -11.9 | -8.1 | programme d'origine |
| 4 | -11.8 | -7.9 | programme vide chargé : même biais |
| 5 | -12.2 | -9.0 | |
| 6 | -24.5 | -5.3 | |
| 7 | -12.9 | -9.9 | |
| 8 | -15.9 | -11.4 | |
| 9 | -7.2 | -8.5 | |
| 10 | -11.8 | -8.1 | |
| 11 | -3.1 | -6.6 | |
| 12 | -3.4 | -11.3 | |
| 13 | -7.7 | -14.5 | |

Biais de la mesure, variable d'une lecture à l'autre, écart-type faible (0.2 à 0.5 mm/s) pendant une lecture.

**Cause : le câble de charge.** Câble débranché (après redémarrage du robot), 3 lectures de 3 s : **exactement 0,0 mm/s** sur les deux roues, sans aucune variation. Le biais n'apparaît que pendant la charge : faire les essais câble débranché.

## Capteurs avant selon la distance (boîte blanche, en face du capteur du milieu)

Distance du bord avant de la coque à la boîte, mesurée à la règle (±5 mm).

| Distance (cm) | prox[0] | prox[1] | prox[2] milieu | prox[3] | prox[4] |
|---|---|---|---|---|---|
| 2 (à la règle) | 0 | 4059 | 4290 | 4332 | 0 |
| 4 | 0 | 2965 | 3628 | 3259 | 0 |
| 6 | 0 | 2140 | 2845 | 2614 | 0 |
| 8 | 0 | 1510 | 2416 | 2092 | 0 |
| 10 | 0 | 0 | 2051 | 0 | 0 |
| 12 | 0 | 0 | 1702 | 0 | 0 |
| 14 | 0 | 0 | 0 | 0 | 0 |
| ~2 (au jugé, non retenu) | 0 | 3753 | 4258 | 4121 | 0 |
| ~7 au jugé, en fait ~9 | 0 | 0 | 2030 | 1626 | 0 |

Portée : le capteur du milieu voit la boîte jusqu'à 12 cm (1702) et plus rien à 14 cm.
Les valeurs ne descendent pas sous ~1500 : en dessous, le capteur renvoie directement 0.

## Capteurs de sol

| Situation | Gauche | Droit |
|---|---|---|
| Posé sur la table | 914 à 922 | 588 à 602 |
| Soulevé de 5 cm | 63 | 42 |
| Soulevé d'environ 10 cm | 24 | 15 |

Le capteur droit lit environ 35 % de moins que le gauche sur la même surface.
Seuil `GROUND_LIFTED_THRESHOLD = 100` : entre « soulevé de 5 cm » (≤ 63) et « posé » (≥ 588). Soulevé de moins de 5 cm, le robot peut ne pas être détecté. À vérifier sur du noir.

## Ligne droite (consigne 100 / 100 unités, 10 s, câble lâche)

| Mesure | Valeur |
|---|---|
| Durée réelle | 10,08 s |
| Distance parcourue (règle) | 350 mm |
| Déviation sur le côté | aucune visible |
| Vitesse mesurée, gauche | 98,8 ± 7,1 unités |
| Vitesse mesurée, droite | 98,0 ± 7,8 unités |

- `SPEED_FACTOR` = 350 / 10,08 / 98,4 = **0,353 mm/s par unité** (0,347 si on divise par la consigne 100). Précision : ±5 mm sur 350, soit ±1,5 %.
- `WHEEL_SPEED_STD_MMS` = 7,5 × 0,353 = **2,6 mm/s**.
- `LEFT_CORRECTION` = **1,0** (pas de déviation).
- En mouvement, la vitesse mesurée suit la consigne à 2 % près : le biais vu à l'arrêt (jusqu'à −25 mm/s) n'apparaît pas.

## Rotation sur place (consigne -80 / +80 unités, 21 s)

| Mesure | Valeur |
|---|---|
| Durée réelle | 21,05 s |
| Vitesse mesurée | gauche -79,5 ± 3,5, droite 79,4 ± 2,3 unités |
| Angle calculé (odométrie, empattement 95 mm, facteur 0,353) | 701° |
| Angle réel (photo : 2 tours moins 35° entre l'avant du robot et le bâtonnet) | 685° ± 5° |

- `WHEEL_BASE_MM` = 95 × 701 / 685 = **97,2 mm** (96,5 à 97,9 mm pour ±5°).
- Angle mesuré sur la photo : bord arrière du robot à 33 à 34° de l'horizontale, bâtonnet incliné de 1° de l'autre côté.

## Boucle de commande : durée réelle d'un pas

- Avec attente de confirmation à chaque envoi (set_variables) : **212 ms** par pas au lieu de 100 ms (moteurs 105 ms + LEDs 105 ms).
  Conséquence : l'odométrie (qui comptait 100 ms) sous-estimait distances et angles d'un facteur 2 ; premier carré : 80 cm au lieu de 40, demi-tour au lieu d'un quart de tour.
- Correction : envoi sans attendre la confirmation, LEDs seulement quand la couleur change, odométrie avec la durée réelle du pas. Après : **104 ms** par pas (101 à 105).

## Carré de 40 cm, odométrie seule (K_ALPHA = 3, OMEGA_MAX = 2,5)

| Mesure | Valeur |
|---|---|
| Durée | 23,6 s (simulation : 24,1 s) |
| Fin estimée par l'odométrie | x = 1 mm, y = 13 mm, θ = -89° |
| Fin réelle (2 photos, trou du stylo = centre de l'axe des roues, repères : bâtonnet et un cheveu) | environ 35 mm après le départ dans le sens du dernier côté, 15 mm à l'extérieur du carré ; θ ≈ -90° |
| Erreur de l'odométrie | environ 4 à 5 cm pour 1,6 m parcouru (≈ 3 %) |

Observé : dans les virages, le robot tourne trop puis se recorrige vers la droite (dépassement), coins arrondis.

## Réglage de K_ALPHA (carré de 40 cm, odométrie seule, OMEGA_MAX = 2,5 rad/s)

Dépassement = de combien le cap dépasse la direction du nouveau côté après un virage (estimé par l'odométrie).
Oscillation = écart de cap maximal sur la seconde moitié du côté.

| K_ALPHA | Côté 2 | Côté 3 | Côté 4 | Durée | Fichiers |
|---|---|---|---|---|---|
| 3 | +38°, puis ±28° | +30°, puis ±11° | +37°, puis ±14° | 24,3 s | data/reel/carre_k3.json, .png |
| 3 (2e essai) | +52°, puis ±19° | +34°, puis ±8° | +38°, puis ±14° | 24,6 s | data/reel/carre_k3_bis.json, .png |
| 2 | +23°, puis ±5° | +14°, puis ±4° | +28°, puis ±5° | 23,0 s | data/reel/carre_k2.json, .png |
| 1,5 | +16°, puis ±4° | +13°, puis ±7° | +13°, puis ±6° | 23,0 s | data/reel/carre_k1.5.json, .png |

Écart de la position estimée au chemin (carré) :

| K_ALPHA | Écart max | Écart moyen |
|---|---|---|
| 3 | 23 et 34 mm | 5,2 et 7,4 mm |
| 2 | 21 mm | 4,8 mm |
| 1,5 | 37 mm | 10,2 mm |

**Choix : K_ALPHA = 2.** À 3, le robot oscille après chaque virage ; à 1,5, il dépasse un peu moins en cap mais prend les virages plus larges (jusqu'à 3,7 cm du chemin, à compter dans la marge de la planification). Vidéos : data/reel/video_k3_vs_k2.jpg (trajet de la LED suivi image par image).

K = 3 : après chaque virage, le cap oscille autour de la bonne direction (période ~2 s) et s'amortit lentement ; même la première ligne droite ondule de ±5°.

## Évitement réel : gobelet (3-4 cm de diamètre, 10 cm de haut) à 13 cm devant le robot, sur le premier côté

Réglages : PROX_ENTER 2400, K_PROX 0,03, CLEAR_DISTANCE 200 mm, K_ALPHA 2.

- Le robot a pivoté d'environ 90° vers la droite (au lieu d'environ 45° en simulation), puis a roulé 23 cm tout droit sur le côté avant de revenir au chemin.
- Il est ainsi arrivé dans les objets posés à droite de la table : plusieurs évitements à la suite, carré non terminé en 60 s (data/reel/carre_obstacle.json, .png).
- Cause : le retard entre la commande et le mouvement du vrai robot (communication + moteurs). Le simulateur l'ignorait.
  Avec un retard de 3 pas (0,3 s), le simulateur reproduit le dépassement mesuré dans les virages (K = 3 : +26° simulé contre +30 à +52° réels ; K = 2 : +17° contre +14 à +28°) et les ~23 cm d'écart de l'évitement.
- Correction : CLEAR_DISTANCE 200 → 100 mm. En simulation (avec retard, 15 positions et tailles d'obstacle) : écart latéral moyen 242 → 160 mm, même marge minimale, même nombre de contacts (1 cas limite : gobelet étroit décalé de 2 cm, vu tard parce qu'il passe entre deux capteurs).

### 2e essai (CLEAR_DISTANCE 100 mm, K_PROX 0,03)

Pivot d'environ 128° devant le gobelet, retour vers le chemin qui repasse près du gobelet, une dizaine d'évitements en 60 s, carré non terminé (data/reel/carre_obstacle_2).

### 3e essai (CLEAR_DISTANCE 100 mm, K_PROX 0,015) : réussi

| Mesure | Valeur |
|---|---|
| Résultat | carré terminé en 29,4 s (23 s sans obstacle), arrivée estimée à 1 cm du but |
| Évitements | 2 (le robot revoit le gobelet en revenant vers le chemin, l'évite une 2e fois) |
| Écart latéral maximal | 14 cm |
| Rotation au 1er évitement | jusqu'à -116° (dont ~25° après que plus aucun capteur ne voit le gobelet : retard) |
| Capteurs (data/reel/carre_obstacle_kp015_prox.json) | milieu : 1528 → 3431 en approchant ; puis le gobelet passe sur les capteurs gauches (prox[1] 4371, prox[0] 4955 = très proche du coin avant gauche) |

Choix : K_PROX = 0,015.

### 4e essai (K_PROX 0,015, PROX_ENTER 2000 au lieu de 2400) : pas mieux

- Carré terminé en 31,6 s, mais 3 évitements, écart latéral maximal 21 cm (contre 14 cm à 2400).
- Le capteur avant gauche monte quand même à 4880 : le robot passe aussi près du gobelet.
- Le déclenchement plus tôt ne sert à rien : entre le déclenchement (x = 42 mm) et le début du virage dans l'odométrie (x = 74 mm), il s'écoule environ 0,5 s.
- Les valeurs des capteurs reçues sont souvent identiques deux pas de suite : elles semblent n'arriver que toutes les 0,2 s environ. À vérifier : c'est peut-être une partie du retard.

Choix : PROX_ENTER reste à 2400.

## Fréquence des mesures reçues

- Avec `wait_for_variables` (variables transmises par le Thymio Device Manager) : vitesses des roues reçues **15 fois en 3 s, soit toutes les ~200 ms**. Dans la boucle à 10 Hz, une mesure sur deux était identique à la précédente.
- tdmclient n'a pas de réglage pour ça : c'est le Device Manager qui fixe ce rythme.
- Solution : un petit programme Aseba sur le Thymio envoie lui-même ses mesures avec un événement (`emit`) toutes les 50 ms. Mesuré : **57 événements en 3 s (toutes les 55 ms)** ; dans la boucle, 1 mesure sur 29 seulement est identique à la précédente.

### 5e essai : mêmes réglages (2400, K_PROX 0,015, CLEAR 100), mesures reçues toutes les 50 ms

| | Essai 3 (mesures toutes les 200 ms) | Essai 5 (toutes les 50 ms) |
|---|---|---|
| Durée du carré | 29,4 s | 26,2 s |
| Évitements | 2 | 2 |
| Écart latéral max | 142 mm | 133 mm |
| Rotation max au 1er évitement | -116° | -84° |
| Délai déclenchement → début du virage | ~0,5 s | ~0,2 s |
| Capteur avant gauche max | 4955 | 4812 |

Le robot réagit plus vite et pivote moins. Il passe encore près du gobelet par son coin avant gauche (le gobelet glisse sur le côté pendant qu'il tourne).
