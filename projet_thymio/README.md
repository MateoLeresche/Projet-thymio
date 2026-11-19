# Projet Thymio – Navigation autonome

Robot Thymio qui va d'un point A à un point B sur une surface blanche délimitée par un carré noir, en évitant les obstacles. Caméra fixe au-dessus du terrain (QR code sur le Thymio).

## Équipe et modules

| Module | Dossier | Responsable | Rôle |
|---|---|---|---|
| Vision | `vision/` | Sujith | caméra, QR code, obstacles, limites du terrain |
| Filtre | `filtre/` | Matéo | filtre de Kalman, estimation de pose |
| Planification | `planification/` | Robin | grille d'occupation, A* |
| Contrôle | `controle/` | Matthieu | commande moteurs (tdmclient), évitement local, intégration |


## Règles d'or

1. **Chacun ne modifie que son dossier.** Cela évite presque tous les conflits Git.
2. **`interfaces.py` est partagé** : prévenir avant de modifier. Le détail du contrat est dans `docs/CONTRAT_INTERFACES.md`.
3. Un module ne connaît que les structures de `interfaces.py`, jamais l'implémentation des autres.
4. Pas de pixels hors de `vision/`, pas d'unités Thymio brutes hors de `controle/`.
5. Si possible chaque module est testable seul.

## Installation

```bash
git clone <URL_DU_DEPOT>
cd projet_thymio
python -m venv .venv
source .venv/bin/activate        # Windows : .venv\Scripts\activate
pip install -r requirements.txt
pytest                           # doit afficher que tous les tests passent
```

## Workflow Git (5 lignes)

```bash
git pull                              # 1. avant de commencer
# ... tu travailles dans TON dossier ...
git add .                             # 2. tout ce que tu as modifié
git commit -m "filtre: ajout du simulateur"   # 3. message court : "module: quoi"
git pull                              # 4. récupérer le travail des autres
git push                              # 5. envoyer
```

Conseils :

- **Ne jamais utiliser `git push --force`.**
- En cas de conflit ou de doute pas hésitez à demander
- Pas de GitHub Desktop ? Les boutons *Commit* / *Push* / *Pull* de VS Code font la même chose.

## Branches

- **Semaine 1** : tout le monde pousse sur `main`, dans son dossier.
- **Semaine 2 (intégration)** : une branche `integration` pour les tests de bout en bout ; `main` reste toujours fonctionnel (le code qui marche pour la démo).

## Structure

```
interfaces.py          contrat partagé
filtre/                Matéo : Kalman (maths pures + boucle temporelle)
vision/                Sujith
planification/         Robin
controle/              Matthieu
simulation/            simulateur (vérité terrain, bruit, pertes de QR)
evaluation/            métriques (erreur, NIS, graphes)
tests/                 tests automatiques
data/                  enregistrements (gros fichiers exclus de Git)
notebooks/             rapport et expériences
docs/                  contrat d'interfaces, journal de calibration
```

## Journal de calibration

Toute valeur mesurée (facteur Thymio → mm/s, empattement, bruit caméra, Q, taille de cellule, Ts) est notée dans `docs/CONTRAT_INTERFACES.md` (section 7) avec sa méthode et sa date. Utile pour le rapport et les questions du prof.
