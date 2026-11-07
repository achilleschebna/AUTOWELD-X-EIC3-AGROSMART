# Outil de soudage fictif (torche virtuelle)

Auteur : Isaac Denis LAPOINTE, membre n°07 de l'équipe AGROSMART-X
Fichier principal : `robots/tools/welding_torch.xacro`

## 1. Objectif

Fournir un outil virtuel à fixer à l'extrémité d'un bras robotique, avec un point de travail (TCP) bien défini. Aucune soudure réelle n'est simulée : la torche, le fil et l'arc sont uniquement des éléments visuels.

## 2. Description du modèle

La torche est une macro Xacro paramétrable, `welding_torch`, qui prend trois paramètres : `prefix` (préfixe des noms, pour réutiliser la torche sur plusieurs robots), `parent` (le repère sur lequel elle se fixe, par exemple `a_tool0`) et un bloc `origin` (décalage de montage).

| Pièce | Lien URDF | Forme | Dimensions (m) |
|---|---|---|---|
| Adaptateur | `<prefix>torch_adapter` | cylindre | rayon 0,045, longueur 0,03 |
| Manche | `<prefix>torch_body` | cylindre | rayon 0,025, longueur 0,12 |
| Col incliné (35°) | `<prefix>torch_neck` | cylindre | rayon 0,012, longueur 0,10 |
| Buse (cuivre) | `<prefix>torch_nozzle` | cylindre | rayon 0,016, longueur 0,05 |
| Fil d'apport | `<prefix>torch_wire` | cylindre | rayon 0,0015, longueur 0,02 |
| Point de travail (TCP) et arc virtuel | `<prefix>torch_tcp` | sphère orange translucide | rayon 0,008 |

Toutes les articulations de la torche sont de type `fixed`. Les pièces principales (adaptateur, manche, col, buse) ont une forme de collision ; le fil et le TCP n'en ont pas, car ils sont purement visuels. L'angle d'inclinaison du col est défini par la propriété Xacro `tilt` (35°).

Le repère `torch_tcp` est le point de travail à atteindre pour chaque point ou segment de soudage fictif.

## 3. Utilisation

Inclure le fichier et appeler la macro sur le repère d'outil d'un bras :

```xml
<xacro:include filename="tools/welding_torch.xacro"/>

<xacro:welding_torch prefix="a_" parent="a_tool0">
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:welding_torch>
```

## 4. Tests réalisés

| Test | Commande | Résultat |
|---|---|---|
| Génération de l'URDF avec le bras de l'équipe | `xacro robots/arm_with_torch_test.urdf.xacro \| grep -c "<link name"` | 12 liens (6 pour le bras, 6 pour la torche) |
| Affichage 3D | `robot_state_publisher`, `joint_state_publisher_gui` et `rviz2`, avec Fixed Frame `world` et RobotModel sur `/robot_description` | Bras et torche affichés ; la torche suit les mouvements des articulations |

Captures : voir `screenshots/` (fichiers `03_bras_et_torche.png`, `03b_torche_gros_plan.png`, à adapter aux noms réellement déposés).

## 5. Limites connues

- Les masses et inerties sont des valeurs approximatives, non calculées.
- Les formes sont des cylindres et sphères simples, sans maillage 3D.
- L'arc virtuel est une sphère fixe : il n'est pas animé et ne s'allume pas selon le cycle de soudage.
- Le repère `tool0` du fichier `bras_3ddl.xacro` contient déjà une petite torche noire, qui se superpose à celle-ci : à retirer pour ne garder qu'un repère vide.
- La torche n'a pas encore été testée dans Gazebo avec la commande ros2_control.
- Les points et segments de travail ne sont pas encore définis : ils dépendent de la position de la pièce à souder.

## 6. Pistes d'amélioration

- Définir les points et segments de travail du TCP.
- Animer l'arc virtuel pendant les phases de soudage.
- Utiliser un maillage 3D plus réaliste pour la torche.
