# Robot A – Conception des trajectoires de soudure

Simulation d'un robot de soudage 6 axes (robot A) dans **Gazebo Classic** avec **ROS 2**.
Le robot suit une trajectoire de soudure le long d'un joint placé sur une table.

---

## Sommaire

- [Description](#description)
- [Structure du projet](#structure-du-projet)
- [Prérequis](#prérequis)
- [Installation](#installation)
- [Utilisation](#utilisation)
- [Trajectoire de soudure](#trajectoire-de-soudure)
- [Personnalisation](#personnalisation)
- [Dépannage](#dépannage)
- [Auteur](#auteur)

---

## Description

Le projet contient :

- une **cellule de soudage** : un robot A à 6 articulations et une table avec un joint à souder (en rouge) ;
- un **programme ROS 2** qui calcule et envoie une trajectoire de soudure au robot ;
- un cycle automatique en boucle : **home → approche → soudure → retrait → home**.

---

## Structure du projet

```
robot_a_trajectoire/
├── README.md
├── src/                 # Code Python, packages ROS 2
│   └── trajectoire_robot_a.py
├── simulation/          # Mondes et fichiers Gazebo
│   └── Conception_des_trajectoires.world
├── robots/              # URDF, Xacro, meshes
└── config/              # Configurations et trajectoires
```

| Dossier       | Contenu                                                         |
|---------------|-----------------------------------------------------------------|
| `src/`        | Nœuds ROS 2 en Python (génération et envoi des trajectoires)    |
| `simulation/` | Mondes Gazebo (`.world`, `.sdf`) : robot, table, éclairage      |
| `robots/`     | Description des robots : URDF, Xacro, modèles 3D (meshes)       |
| `config/`     | Paramètres : contrôleurs, points de trajectoire, réglages       |

---

## Prérequis

- Ubuntu 22.04 (natif ou WSL2)
- ROS 2 (Humble recommandé)
- Gazebo Classic 11
- Paquets Gazebo pour ROS 2 :

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs
```

---

## Installation

```bash
# Créer la structure du projet
mkdir -p ~/robot_a_trajectoire/{src,simulation,robots,config}

# Copier les fichiers depuis le dossier Téléchargements de Windows (WSL)
# (guillemets obligatoires : le nom d'utilisateur Windows contient un espace)
SRC="/mnt/c/Users/Ecole Ecolo/Downloads/conception_trajectoires"
cp "$SRC/trajectoire_robot_a.py"            ~/robot_a_trajectoire/src/
cp "$SRC/Conception_des_trajectoires.world" ~/robot_a_trajectoire/simulation/
cp "$SRC/README.md"                         ~/robot_a_trajectoire/

# Vérifier
ls -R ~/robot_a_trajectoire
```

---

## Utilisation

Il faut **deux terminaux** Ubuntu. Taper les commandes **une par une** (Entrée après chaque ligne).

### Terminal 1 – Lancer la simulation Gazebo

```bash
killall -9 gzserver gzclient gazebo   # ferme un ancien Gazebo (« no process found » = normal)
source /opt/ros/humble/setup.bash
cd ~/robot_a_trajectoire
gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so simulation/Conception_des_trajectoires.world
```

### Terminal 2 – Lancer la trajectoire

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_a_trajectoire
python3 src/trajectoire_robot_a.py
```

Le terminal affiche `Cycle 1 envoye` et le robot commence à bouger.
Arrêter avec `Ctrl + C`.

### Topics ROS 2

| Topic                            | Type                               | Rôle                          |
|----------------------------------|------------------------------------|-------------------------------|
| `/robot_a/set_joint_trajectory`  | `trajectory_msgs/JointTrajectory`  | Commande de la trajectoire    |
| `/robot_a/joint_states`          | `sensor_msgs/JointState`           | Angles actuels des 6 axes     |

Vérifier les angles en direct :

```bash
ros2 topic echo /robot_a/joint_states
```

---

## Trajectoire de soudure

| Étape           | Description                                      | Durée  |
|-----------------|--------------------------------------------------|--------|
| Home            | Position de repos                                | —      |
| Approche        | Torche 12 cm au-dessus du début du joint         | 3,0 s  |
| Début soudure   | Descente à 12 mm du joint                        | 1,5 s  |
| Soudure         | Suivi du joint sur 30 cm à 5 cm/s, torche verticale | 6,0 s |
| Retrait         | Remontée de 12 cm                                | 1,5 s  |
| Home            | Retour à la position de repos                    | 3,0 s  |

- Durée d'un cycle : environ **16 s**, puis le cycle recommence.
- Les angles ont été calculés par **cinématique inverse**, avec une précision inférieure à 0,2 mm sur le joint.
- La trajectoire est envoyée à **50 Hz**, la même fréquence que `<update_rate>` du plugin.

---

## Personnalisation

Les points de la trajectoire sont dans la liste `KEYFRAMES` du fichier `src/trajectoire_robot_a.py` :

```python
("nom", [j1, j2, j3, j4, j5, j6], duree_en_secondes, mouvement_lisse),
```

- **Ralentir la soudure** : augmenter la durée `0.6` des lignes `soudure_*`.
- **Changer la pause entre cycles** : modifier `PAUSE`.
- Les angles sont en **radians** et doivent respecter les limites des articulations.

| Articulation      | Axe | Limite (rad)   |
|-------------------|-----|----------------|
| `robot_a_joint_1` | Z   | ± 2,97         |
| `robot_a_joint_2` | Y   | ± 2,00         |
| `robot_a_joint_3` | Y   | ± 2,50         |
| `robot_a_joint_4` | X   | ± 3,14         |
| `robot_a_joint_5` | Y   | ± 2,10         |
| `robot_a_joint_6` | X   | ± 3,14         |

---

## Dépannage

| Problème                                         | Solution                                                                 |
|--------------------------------------------------|--------------------------------------------------------------------------|
| Gazebo s'ouvre sur un monde vide                 | Vérifier le nom et le chemin exacts du fichier `.world`                  |
| Le terminal 2 affiche `En attente du plugin...`  | Relancer Gazebo avec `-s libgazebo_ros_init.so`                          |
| `Address already in use` / `Gazebo is already running` | Fermer l'ancien Gazebo : `killall -9 gzserver gzclient gazebo`     |
| `can't open file ... No such file or directory`  | Fichier absent : refaire l'étape [Installation](#installation)           |
| La commande `gazebo` ou `python3` ne fait rien    | Les lignes ont été collées ensemble : les taper une par une              |
| Le robot ne bouge pas, erreur de plugin           | Installer `ros-humble-gazebo-ros-pkgs`                                   |
| Messages `ALSA lib ...` au démarrage             | Sans danger (absence de carte son, fréquent sous WSL), à ignorer        |
| Le robot retombe                                 | Vérifier que `<gravity>0</gravity>` est présent sur les liens du bras    |

---

## Auteur

**cole_colo** – Projet AutoWeld
