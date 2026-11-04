# AUTOWELD-X - Cellule robotisée de soudage simulée

Équipe AGROSMART-X - EIC 3.0 - Professeur : Carlos Yfrazin - Établissement : EIC

Simulation 3D d'une cellule robotisée de **soudage fictif** : trois bras robotiques travaillent automatiquement, l'un après l'autre, sur une pièce métallique, puis s'arrêtent. Aucun soudage réel n'est réalisé : l'arc, les étincelles et le cordon sont entièrement virtuels.

## Membres de l'équipe

| N° | Nom | Rôle prévu dans le cahier des charges |
|----|-----|----------------------------------------|
| 01 | ACHILLE Schebna | Coordination et intégration |
| 02 | BAZELAIS Yodlène | Analyse de la référence vidéo |
| 03 | CHÉRY Esther | Modélisation du châssis / support |
| 04 | DUVERT Stanley Kenlove | Conception des trajectoires - robot A |
| 05 | JÉRÔME Samandina | Commande et trajectoires - robot B |
| 06 | LAMARRE Jean Alex | Commande et trajectoires - robot C |
| 07 | LAPOINTE Isaac Denis | Modélisation des outils de soudage fictif |
| 08 | LINERSON Eden | Placement des pièces et repères |
| 09 | LOUISSAINT Enock | Coordination multi-robots et collisions |
| 10 | NAÏVELANDE Fortunat | Scénarios automatisés et séquenceur |
| 11 | PIERRE Hervens | Programmation Python / logique de contrôle |
| 12 | PIERRE Ovaldo | Tests, mesures et qualité |
| 13 | SAINT JUSTE Jn Roothsviski | README, GitHub, captures et soutenance |

Ces rôles sont ceux proposés par le cahier des charges. Ce que chacun a réellement réalisé est détaillé dans [`docs/contributions.md`](docs/contributions.md).

## Objectif et architecture

**Objectif.** Simuler en 3D une cellule industrielle où plusieurs robots à bras articulés effectuent, de façon automatique et sécurisée, une opération de soudage fictif sur une même pièce, sans collision.

**Disposition de la cellule.** La pièce est au centre, sur un socle. Chaque robot est placé à 0,8 m de la pièce et tourné vers elle :

```
              Robot C (nord)
                   |
 Robot A  ------ [PIÈCE] 
 (ouest)           |
              Robot B (sud)
```

- **Robots A, B et C** : trois bras identiques à 3 articulations (rotation de la base, épaule, coude), terminés par une torche de soudage fictive.
- **Pièce à souder** : un bloc métallique de 30 x 30 x 40 cm posé sur un socle de 50 x 50 x 20 cm, avec un cordon de soudure repéré sur chacune des trois faces utilisées (ouest pour A, sud pour B, nord pour C).

**Chaîne logicielle.**

```
 Modèles Xacro (robots/)
        |
        v
 robot_state_publisher --> Gazebo Harmonic (simulation 3D)
                                ^
                                | plugin gz_ros2_control
                                v
                  ros2_control : joint_state_broadcaster
                                 robot_a_controller, robot_b_controller, robot_c_controller
                                ^
                                | trajectoires (JointTrajectory)
                      sequence_soudure.py (scénario Python)
```

## Logiciels et installation

| Logiciel | Version / remarque |
|----------|--------------------|
| Système | Windows avec WSL 2, Ubuntu 24.04 |
| ROS 2 | Jazzy Jalisco |
| Simulateur | Gazebo Harmonic (`gz-harmonic`, `ros-jazzy-ros-gz`) |
| Commande des robots | `ros2_control`, `ros2_controllers`, `gz_ros2_control` |
| Planification | MoveIt 2 (installé, non utilisé pour l'instant - voir Limites) |
| Langage | Python 3 |
| Compilation | colcon |

**Installation (dans un terminal Ubuntu 24.04).**

1. Installer WSL 2 et Ubuntu 24.04 (PowerShell en administrateur) :
   ```
   wsl --install -d Ubuntu-24.04
   ```
2. Installer ROS 2 Jazzy en suivant la documentation officielle : <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>
3. Installer Gazebo Harmonic en ajoutant le dépôt de paquets Gazebo (voir la documentation officielle de Gazebo), puis les paquets du projet :
   ```bash
   sudo apt install -y gz-harmonic ros-jazzy-ros-gz ros-jazzy-moveit \
     ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
     ros-jazzy-gz-ros2-control ros-jazzy-xacro \
     python3-colcon-common-extensions git
   ```
4. Récupérer et compiler le projet. **Clonez-le dans le dossier personnel d'Ubuntu** (par exemple `~`), pas dans `/mnt/c/...` : le paquet utilise des liens symboliques, qui ne fonctionnent pas sur le disque Windows.
   ```bash
   source /opt/ros/jazzy/setup.bash
   git clone https://github.com/<compte>/AUTOWELD-X-EIC3-AGROSMART.git
   cd AUTOWELD-X-EIC3-AGROSMART
   colcon build
   source install/setup.bash
   ```
5. Vérifier l'installation :
   ```bash
   printenv ROS_DISTRO     # doit afficher : jazzy
   gz sim --version        # Gazebo Harmonic
   ```

## Lancer la simulation

**Terminal 1 - la cellule 3D :**
```bash
cd AUTOWELD-X-EIC3-AGROSMART
source install/setup.bash
ros2 launch autoweld_x_control cell_sim.launch.py
```
Gazebo s'ouvre avec la pièce et les trois bras verticaux. Les contrôleurs se lancent l'un après l'autre : comptez environ une minute.

**Terminal 2 - vérifier les contrôleurs :**
```bash
ros2 control list_controllers
```
Quatre contrôleurs doivent être `active` : `joint_state_broadcaster`, `robot_a_controller`, `robot_b_controller`, `robot_c_controller`.

**Terminal 2 - lancer le scénario automatique :**
```bash
python3 src/autoweld_x_control/autoweld_x_control/sequence_soudure.py cycle
```

| Commande | Effet |
|----------|-------|
| `... sequence_soudure.py cycle` | Les trois robots soudent l'un après l'autre (environ 16 s chacun), puis le programme s'arrête (valeur par défaut). |
| `... sequence_soudure.py approche` | Les trois robots se placent en position d'approche, en même temps. |
| `... sequence_soudure.py repos` | Les trois robots reviennent à la verticale, en même temps. |

**Résultat attendu.** Chaque torche s'approche de sa face de la pièce, parcourt lentement le cordon rouge, se retire et revient à la verticale. Le terminal affiche la réservation de la zone (`[ZONE] reservee par robot_a`, puis `liberee`) et se termine par `Cycle termine : tous les robots sont au repos.` Arrêtez la simulation avec **Ctrl + C**.

**En cas de fenêtre Gazebo noire** (pas d'accélération graphique sous WSL 2) :
```bash
LIBGL_ALWAYS_SOFTWARE=1 QT_QPA_PLATFORM=xcb ros2 launch autoweld_x_control cell_sim.launch.py
```

## Structure du dépôt

```
AUTOWELD-X-EIC3-AGROSMART/
├── README.md
├── src/
│   └── autoweld_x_control/            # paquet ROS 2 (Python)
│       ├── autoweld_x_control/
│       │   └── sequence_soudure.py    # scénario automatique + coordination
│       ├── launch/cell_sim.launch.py  # lancement de la simulation
│       ├── robots  -> ../../robots    # liens symboliques vers les dossiers
│       ├── config  -> ../../config    # ci-dessous, vus par le paquet
│       └── worlds  -> ../../simulation
├── robots/
│   ├── bras_3ddl.xacro                # modèle réutilisable d'un bras
│   ├── piece_a_souder.xacro           # socle, pièce et cordons
│   └── cellule.urdf.xacro             # assemblage de la cellule
├── config/controllers.yaml            # contrôleurs ros2_control
├── simulation/                        # mondes Gazebo
├── screenshots/                       # captures d'écran
├── tests/                             # procédures et résultats de tests
├── docs/                              # contributions, architecture
├── presentation/                      # diapositives
└── demo/                              # lien de la vidéo
```

## Explication du code et choix techniques

**Modèle d'un bras (`robots/bras_3ddl.xacro`).** Un bras est décrit une seule fois sous forme de *macro* Xacro, avec un préfixe (`robot_a_`, `robot_b_`, `robot_c_`) et une position. Ajouter un robot revient à ajouter une ligne dans la scène. Chaque bras comprend un socle fixe et trois articulations rotatives : `joint_1` (rotation de la base, axe vertical, ±3,14 rad), `joint_2` (épaule, ±1,57 rad) et `joint_3` (coude, ±2,0 rad), puis la torche fictive. Les noms d'articulations suivent la convention `robot_x_joint_n`.

**Pièce et scène (`piece_a_souder.xacro`, `cellule.urdf.xacro`).** La pièce est un objet fixe, avec des surfaces de collision, qui porte les trois cordons (éléments purement visuels). La scène assemble la pièce et les trois bras, et déclare le plugin `gz_ros2_control` qui relie Gazebo aux contrôleurs.

**Contrôle (`config/controllers.yaml`).** Chaque robot a son propre contrôleur de trajectoire (`joint_trajectory_controller`) en commande de position. Un contrôleur par robot permet de commander chaque bras indépendamment avec des trajectoires synchronisées.

**Lancement (`launch/cell_sim.launch.py`).** Le fichier démarre Gazebo, publie la description de la cellule, fait apparaître la scène, relie l'horloge de simulation à ROS 2, puis lance les contrôleurs l'un après l'autre (chaque lancement attend la fin du précédent, ce qui évite les conflits de démarrage).

**Scénario (`sequence_soudure.py`).** Un nœud ROS 2 envoie à chaque robot une trajectoire à cinq points : approche, début du cordon, fin du cordon, retrait, retour au repos. Il attend en **temps de simulation** (et non en temps réel), de sorte que le scénario reste correct même si la simulation est ralentie. À la fin de chaque cycle, il vérifie que le robot est bien revenu au repos, puis le programme se termine.

**Calcul des poses.** Les angles d'approche et de soudure ont été calculés par cinématique inverse d'un bras plan à deux segments (loi des cosinus), à partir de la géométrie réelle du modèle : torche à 3 cm de la face à souder, cordon parcouru en tournant la base de ±0,09 rad (environ ±5,7 cm).

**Implantation des robots.** Les robots sont à 0,8 m de la pièce, chacun tourné vers elle. Cette implantation est le résultat du calcul : plus près, le coude dépassait sa limite articulaire ; avec un robot face à l'ouest, la base atteignait la limite de ±3,14 rad. Les robots B et C se font face de part et d'autre de la pièce, ce qui crée une vraie zone partagée à coordonner.

**Organisation du dépôt.** Les modèles, la configuration et les mondes sont à la racine du dépôt, conformément au cahier des charges, et le paquet ROS 2 les voit par des liens symboliques.

### Évitement des collisions et coordination

- **Vérification géométrique avant exécution.** Les segments des bras sont assimilés à des cylindres, et les distances entre les bras, la pièce, le socle et les autres bras sont calculées le long de chaque trajectoire. Résultats : la torche reste à environ 3 cm de la pièce (1 cm de marge dans le modèle prudent), toutes les autres marges dépassent 8 cm, et elles dépassent 68 cm avec les bras au repos.
- **Limites articulaires.** Toutes les poses respectent les limites déclarées dans le modèle.
- **Zone partagée.** Une classe `ZonePartagee` joue le rôle de verrou : un robot doit réserver la zone avant de travailler et la libérer ensuite. Un seul robot à la fois travaille donc autour de la pièce, ce qui supprime le risque de collision entre bras.
- **Approche progressive.** Chaque robot arrive d'abord en retrait et au-dessus de la pièce, puis avance lentement vers le cordon, et se retire par le même chemin.

## Captures d'écran et vidéo

Les captures sont dans le dossier [`screenshots/`](screenshots/) :

- [Installation de ROS 2 et Gazebo](screenshots/01_installation_ros2.png)
- [Gazebo avec un monde vide](screenshots/02_gazebo_monde_vide.png)
- [Premier bras à la verticale](screenshots/03_bras_robot_a_vertical.png)
- [Premier bras plié](screenshots/04_bras_robot_a_plie.png)
- [Les trois bras au repos](screenshots/05_trois_bras_repos.png)
- [Scène complète avec la pièce](screenshots/06_scene_complete.png)
- [Un robot en cours de soudure](screenshots/07_robot_en_soudure.png)
- [Coordination et fin de cycle dans le terminal](screenshots/08_coordination_terminal.png)

**Vidéo de démonstration (3 à 5 minutes) :** [à compléter : lien de la vidéo]

## Contributions, limites et améliorations

**Contributions.** Voir [`docs/contributions.md`](docs/contributions.md) : chaque membre y décrit ses travaux, avec ses commits et ses captures.

**Limites rencontrées.**

- MoveIt 2 est installé, mais n'est pas encore utilisé pour planifier les trajectoires : les mouvements sont des séquences de poses prédéfinies et calculées.
- L'évitement de collisions repose sur une vérification géométrique simplifiée et sur l'exécution séquentielle des robots. Il n'y a pas de détection dynamique des obstacles en temps réel.
- Les robots travaillent l'un après l'autre : le travail simultané sous contrôle de zone n'est pas implémenté.
- Le soudage est entièrement fictif : aucun arc, aucune étincelle ni aucun contact physique avec la pièce ne sont simulés, et les cordons sont représentés visuellement.
- Aucun module d'intelligence artificielle ni de vision par caméra n'est intégré (partie bonus).
- Sous WSL 2, l'affichage de Gazebo peut être ralenti selon les ordinateurs, car l'accélération graphique n'est pas toujours disponible.

**Améliorations possibles.**

- Intégrer MoveIt 2 pour planifier les trajectoires et éviter les obstacles automatiquement.
- Autoriser le travail simultané de plusieurs robots grâce à des réservations de sous-zones.
- Simuler le contact et le dépôt de matière le long du cordon (étincelles, cordon progressif).
- Ajouter une caméra et un module de détection pour repérer la pièce et ses points de soudure.
- Ajouter des tests automatiques (poses, limites articulaires, absence de collision).

## Références

- Vidéo de référence de l'enseignant : <https://youtu.be/7iWzFTzYleg>
- Documentation ROS 2 Jazzy : <https://docs.ros.org/en/jazzy/>
- Documentation `gz_ros2_control` (Jazzy) : <https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html>
- Documentation Gazebo : <https://gazebosim.org/docs>
