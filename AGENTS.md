# Consignes du projet cam_imu_calib

## Mission et état

Développer un outil exclusivement CLI de calibration caméra–caméra et caméra–IMU,
avec extrinsèques, synchronisation temporelle, rolling shutter, diagnostic des
données et rapport PDF. Lire `README.md` pour l'état réel, puis uniquement les
documents et modules utiles à la modification. L'architecture proposée est dans
`docs/architecture.md` ; les étapes vérifiables dans `docs/roadmap.md`.

Le contrôle des datasets et la détection ont une première implémentation. Ne pas
présenter les solveurs ou commandes proposés comme disponibles avant leur validation.
Lire `docs/validation.md` pour les preuves et limites des tests existants.

Choix utilisateur acquis : Linux, données existantes `camN/<timestamp_ns>.png` et
`imu0/imu0.csv`, en-tête `timestamp_ns,wx,wy,wz,ax,ay,az`, unités rad/s et m/s²,
horloge commune et offsets à affiner. Ne pas réintroduire CSV caméra, manifeste
ou `rig.yaml` obligatoire. Commandes validées : `calib_cameras` et `calib_imu_camera`.
Caméra–caméra : initialiser automatiquement le placement relatif depuis les
observations, sans placement théorique fourni par l'utilisateur ni prior mécanique.
L'utilisateur pourra comparer lui-même les extrinsèques exportées à son montage.
Le modèle RS/IMU proposé est détaillé dans `docs/rolling-shutter.md`.
Réutiliser `line_delay` du camchain (secondes/ligne, convention Kalibr), fixé par
défaut en VI. Aucun sensors.yaml requis ; raffinage RS seulement sur demande explicite.
Le cœur reste au format natif. L’utilisateur a autorisé un convertisseur ROS1
séparé pour extraire les deux bags 512/1024 de Downloads dans `dataset/`.
Oxford Spires 2024-03-19 : l'inventaire local trouve uniquement des images avec
`header.stamp` nul et du LiDAR, sans IMU. L'utilisateur a choisi d'attendre des
timestamps capteur valides : ne pas substituer les temps d'enregistrement des bags
pour cet export. Voir `docs/recorded-datasets.md` pour l'état de cette importation.
Oxford Spires 2024-10-29 : l'utilisateur confirme la même AprilGrid que DTVI
512/1024. Réutiliser `examples/dtvi_aprilgrid.yaml` (6×6, tags 0,088 m,
espacement 0,3, tag36h11, bordure historique 2, premier ID 0).
Ne pas versionner ces données privées. MCAP et lecteur ROS intégré restent hors périmètre.

## Méthode d'itération

- Répondre en français ; utiliser l'anglais pour les identifiants du code.
- Examiner `git status` et les consignes applicables avant de modifier des fichiers.
  Préserver les changements de l'utilisateur.
- Définir brièvement le résultat observable et les critères de réussite. Pour un
  changement local, éviter de refaire l'étude de tout le projet.
- Utiliser `rg` et lire des portions ciblées. Grouper les lectures indépendantes.
  Réutiliser les conclusions sourcées ; revérifier les versions lors d'une mise à jour.
- Préférence explicite utilisateur : modifier les fichiers existants par petits
  patches ciblés ; éviter leurs réécritures complètes et les lectures redondantes.
- Résoudre les choix réversibles courants sans demander une confirmation. Poser
  seulement les questions dont la réponse change les données, les conventions,
  l'architecture ou le résultat attendu ; avancer sur les travaux indépendants.
- Conserver dans les documents les décisions, hypothèses et preuves utiles,
  pas une transcription du raisonnement interne. Pour une décision structurante :
  problème, choix, alternative principale, conséquence, test qui pourrait l'invalider.
- Livrer par tranches fonctionnelles. Aucun faux résultat, transformation identité
  de substitution, score inventé ou option silencieusement ignorée.
- Tester au niveau du risque. Exécuter les tests ciblés, puis les contrôles requis ;
  ne pas répéter les mêmes tests sans nouveau changement ou résultat à expliquer.
- Pour les changements de lecture, détection et futurs solveurs, réutiliser les
  enregistrements privés `dataset/512` et `dataset/1024` lorsqu’ils sont présents.
  Le protocole est dans `docs/recorded-datasets.md` ; ils partagent timestamps/IMU
  et ne valent pas deux acquisitions indépendantes. Ne pas modifier le YAML fourni
  `dataset/dtvi_aprilgrid.yaml` ; la variante explicite est `.resolved.yaml`.
- Terminer par ce qui fonctionne, les vérifications réellement exécutées et les
  limites restantes. Mettre à jour l'état et la prochaine étape dans la roadmap.

## Invariants scientifiques

- `T_A_B` transforme les coordonnées de B vers A : `p_A = T_A_B * p_B`.
  Documenter repères, ordre quaternion et convention de perturbation des covariances.
- SI en interne : mètres, secondes, radians, m/s². Horodatages source en entiers
  64 bits ; temps d'optimisation relatif à une origine commune, en double.
- Kalibr : `T_cam_imu` est IMU vers caméra ;
  `t_imu = t_cam + timeshift_cam_imu`. Tester explicitement les signes.
- La géométrie physique et la famille de la mire sont explicites. Ne jamais
  supposer qu'une AprilGrid historique est interchangeable avec une grille AprilTag 3.
- Le rolling shutter s'évalue au temps de chaque coin dans le repère du capteur
  original. Décrire sens de lecture, référence temporelle, durée de lecture et exposition.
- Fixer les jauges avant d'interpréter un rang ou une covariance. Marginaliser les
  paramètres de nuisance pour les incertitudes des extrinsèques.
- Une convergence ou un faible RMS ne prouve pas une bonne calibration. Conserver
  les rejets et leur motif ; contrôler excitation, couverture et validation indépendante.
- Ne jamais remplacer une incertitude inconnue par zéro. Distinguer convergence,
  observabilité, cohérence du modèle et aptitude à l'usage.

## Implémentation et performances

- C++ moderne : cible C++26 selon support vérifié de la chaîne, repli C++23 explicite.
  CMake par cibles, presets, dépendances verrouillées par version et empreinte.
- Évaluer les modèles/splines de Basalt derrière des adaptateurs étroits ; Basalt
  est un candidat, pas une obligation. Voir `docs/technical-choices.md`.
  Garder l'algèbre indépendante des formats, du CLI et du générateur PDF.
- RAII, ownership explicite, interfaces typées, erreurs avec contexte. Double pour
  l'optimisation ; éviter allocations et conversions d'image dans les boucles critiques.
- Utiliser un budget CPU global : éviter les pools imbriqués AprilTag/OpenCV/BLAS.
  Borner les files d'images et la mémoire ; ne pas charger tout un dataset 4K.
- Toute optimisation doit mesurer durée, mémoire et effet sur la précision.
  Ne pas activer globalement `fast-math` ou imposer `-march=native` aux binaires distribués.
- Cache invalidé par données, mire, options et version du détecteur ; écriture
  atomique et reprise sans réutilisation de résultats partiels non validés.
- Garder la CLI non interactive et le chemin courant à une commande par dataset.
  Ne pas introduire de GUI ou de dépendance ROS dans le cœur.

## Validation requise selon le changement

- Géométrie/solveur : données synthétiques indépendantes avec vérité terrain,
  Jacobiennes contre différences finies, cas dégénérés, cas limites temporels.
- Rolling shutter : temps de ligne, quatre sens de lecture, limite global shutter,
  exposition, crop/redimensionnement, offsets et passage entre segments de spline.
- Exports : sens des transformations, unités et import par un consommateur indépendant.
- Détection : vrais rendus/images, familles et bordures, rotations de tags, IDs,
  coordonnées des coins, bruit/flou, pleine résolution et décimation.
- Parallélisme/cache : résultats stables à tolérance déclarée, concurrence,
  annulation, corruption, changement de configuration et mémoire bornée.
- Ne pas déclarer un test réussi s'il n'a pas tourné. Ne pas écrire de tests
  uniquement pour des changements documentaires sans risque fonctionnel.

## Dépendances et données

Les sources consultées figurent dans `docs/research.md`. Inspecter les API et
licences avant intégration ; ne pas copier du code d'un outil de référence sans
vérifier sa licence. Ne pas versionner de gros datasets ni de résultats utilisateur.
Conserver les petits jeux reproductibles avec provenance et licence. Les seuils
de qualité sont versionnés et justifiés par les expériences, jamais universels.
