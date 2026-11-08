## Isaac Denis LAPOINTE (n°07) : modélisation des outils de soudage fictif

Réalisé :
- Modèle Xacro paramétrable d'une torche de soudage fictive (`robots/tools/welding_torch.xacro`) : adaptateur, manche, col incliné à 35°, buse, fil d'apport et point de travail (TCP) avec arc virtuel, purement visuel.
- Intégration de la torche sur le repère `a_tool0` du bras `bras_3ddl.xacro`, vérifiée par la génération de l'URDF (12 liens) et par l'affichage dans RViz.
- Installation et configuration de l'environnement (WSL 2, ROS 2 Jazzy, Gazebo Harmonic, MoveIt 2).
- Captures d'écran de la torche sur le bras (dossier `screenshots/`).
- Documentation de l'outil : `docs/outil_soudage_fictif.md`.

Commits : 
a_base_link
Link a_base_link with parent joint a_fixation has 1 child joint: a_joint_1. Check/uncheck to show/hide this link in the display.

a_link_1
Link a_link_1 with parent joint a_joint_1 has 1 child joint: a_joint_2. Check/uncheck to show/hide this link in the display.

a_link_2
Link a_link_2 with parent joint a_joint_2 has 1 child joint: a_joint_3. Check/uncheck to show/hide this link in the display.

a_link_3
Link a_link_3 with parent joint a_joint_3 has 1 child joint: a_fixation_outil. Check/uncheck to show/hide this link in the display.

a_tool0
Link a_tool0 with parent joint a_fixation_outil has 1 child joint: a_torch_mount_joint. Check/uncheck to show/hide this link in the display.

a_torch_adapter
Link a_torch_adapter with parent joint a_torch_mount_joint has 1 child joint: a_torch_body_joint. Check/uncheck to show/hide this link in the display.

a_torch_body
Link a_torch_body with parent joint a_torch_body_joint has 1 child joint: a_torch_neck_joint. Check/uncheck to show/hide this link in the display.

a_torch_neck
Link a_torch_neck with parent joint a_torch_neck_joint has 1 child joint: a_torch_nozzle_joint. Check/uncheck to show/hide this link in the display.

a_torch_nozzle
Link a_torch_nozzle with parent joint a_torch_nozzle_joint has 1 child joint: a_torch_wire_joint. Check/uncheck to show/hide this link in the display.

a_torch_tcp
Link a_torch_tcp with parent joint a_torch_tcp_joint has no children. Check/uncheck to show/hide this link in the display. This link has visible geometry but no collision geometry.

a_torch_wire
Link a_torch_wire with parent joint a_torch_wire_joint has 1 child joint: a_torch_tcp_joint. Check/uncheck to show/hide this link in the display. This link has visible geometry but no collision geometry.

world
Root Link world has 1 child joint: a_fixation. This link has NO geometry.


## Hervens PIERRE (n°11) : programmation Python / logique de contrôle

Réalisé :
- Installation et configuration de l'environnement (WSL 2, Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic 8.15.0, MoveIt 2, ros2_control, colcon), vérifiés par des commandes de contrôle.
- Compilation du dépôt complet et lancement de la cellule : les 4 contrôleurs (joint_state_broadcaster et robots A, B, C) sont actifs.
- Module `src/autoweld_x_control/autoweld_x_control/verification.py` : vérifie les limites articulaires, la croissance des temps et la vitesse maximale (1,5 rad/s) d'une trajectoire avant son envoi.
- Tests `tests/test_verification.py` : 8 tests, tous réussis. L'un d'eux contrôle le cycle réel de `sequence_soudure.py` pour les 3 robots.
- Exécution du cycle complet `sequence_soudure.py cycle` : terminé avec succès en 286,9 s (temps mesuré sur ma machine), voir `tests/procedure_de_test.md`.
- Captures d'écran : `screenshots/Hervens_01` à `Hervens_05`.

Limites :
- `verification.py` n'est pas encore branché dans `sequence_soudure.py` (fichier de Fortunat NAÏVELANDE) : il est utilisé par ses tests.
- Pas de journal d'exécution en fichier, pas de travail simultané des robots.

Commits : 6d10660 (module et tests), 6b55038 (captures), branche `hervens-verification`.
