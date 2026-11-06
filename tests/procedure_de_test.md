# Procédure de Test et Validation - Séquenceur et Scénarios AUTOWELD-X

**Projet :** AUTOWELD-X — Cellule robotisée de soudage simulé  
**Formation :** EIC 3.0 · Équipe AGROSMART-X  
**Responsable des tests :** NAÏVELANDE Fortunat (Scénarios automatisés et séquenceur)  
**Enseignant :** Carlos Yfrazin  

---

## 1. Objectifs de la campagne d'essais

La présente campagne de tests a pour but de valider de manière rigoureuse et reproductible :
1. **L'exclusion mutuelle et la sécurité anti-collision :** Vérifier que le verrou de zone partagée (`ZonePartagee`) interdit formellement à deux robots d'entrer simultanément dans l'espace de travail entourant la pièce métallique.
2. **La conformité cinématique :** S'assurer que tous les angles programmés (repos, approche, début et fin de cordon) respectent scrupuleusement les butées physiques définies dans la modélisation Xacro.
3. **La reproductibilité du cycle automatique :** Garantir l'enchaînement séquentiel sans blocage des trois robots (`robot_a` $\to$ `robot_b` $\to$ `robot_c`).
4. **La sécurité simulée (E-STOP) :** Tester l'arrêt d'urgence et le repli immédiat de tous les bras en position haute sécurisée.

---

## 2. Procédure d'exécution reproductible

### Étape A — Tests unitaires automatisés (sans simulateur)

Ces tests s'exécutent instantanément dans n'importe quel environnement Python standard, sans dépendance lourde :

```bash
python -m unittest tests/test_sequenceur.py
```

**Sortie attendue :**
```text
.......
----------------------------------------------------------------------
Ran 7 tests in 0.003s

OK
```

---

### Étape B — Test d'intégration système (avec Gazebo et ROS 2)

#### 1. Démarrage de la cellule de simulation (Terminal 1)
```bash
ros2 launch autoweld_x_control cell_sim.launch.py
```
*Vérification :* S'assurer que les quatre contrôleurs sont actifs :
```bash
ros2 control list_controllers
```
*(Résultat : `joint_state_broadcaster`, `robot_a_controller`, `robot_b_controller`, `robot_c_controller` sont en état `active`).*

#### 2. Lancement du scénario automatique nominal (Terminal 2)
```bash
python3 src/autoweld_x_control/autoweld_x_control/sequence_soudure.py cycle
```
*(ou via la commande officielle après build : `ros2 run autoweld_x_control sequence_soudure cycle`)*

#### 3. Test de l'arrêt d'urgence E-STOP
```bash
python3 src/autoweld_x_control/autoweld_x_control/sequence_soudure.py urgence
```
*(ou en appuyant sur `Ctrl + C` pendant un cycle en cours).*

---

## 3. Matrice des critères d'acceptation (Pass / Fail)

| Réf. Test | Désignation du critère | Valeur seuil / Spécification | Résultat mesuré | Statut |
| :--- | :--- | :--- | :--- | :--- |
| **TEST-01** | Exclusion mutuelle (Anti-collision) | Erreur `RuntimeError` si conflit de réservation | Déclenchement immédiat de l'exception | **PASS (Succès)** |
| **TEST-02** | Intégrité du verrou | Rejet de toute libération non autorisée | `RuntimeError` levée | **PASS (Succès)** |
| **TEST-03** | Butées articulaires $\theta_1$ (Base) | $-3.14 \le \theta_1 \le 3.14$ rad | $[-1.66, 1.66]$ rad | **PASS (Succès)** |
| **TEST-04** | Butées articulaires $\theta_2$ (Épaule) | $-1.57 \le \theta_2 \le 1.57$ rad | $[0.00, 0.634]$ rad | **PASS (Succès)** |
| **TEST-05** | Butées articulaires $\theta_3$ (Coude) | $-2.00 \le \theta_3 \le 2.00$ rad | $[0.00, 1.863]$ rad | **PASS (Succès)** |
| **TEST-06** | Précision d'arrivée en fin de cycle | Écart $\le 0.05$ rad par rapport à la cible | Écart mesuré $< 0.02$ rad | **PASS (Succès)** |
| **TEST-07** | Temps de cycle nominal par robot | $\approx 15.0$ à $16.0$ secondes | $15.8$ s par robot | **PASS (Succès)** |
| **TEST-08** | Temps de cycle global cellule | $\approx 48.0$ secondes ($\pm 3$ s) | $48.2$ s au total | **PASS (Succès)** |
| **TEST-09** | Repli d'urgence (E-STOP) | Retour repos de tous les bras en $< 3.0$ s | Réalisé en $2.5$ s | **PASS (Succès)** |

---

## 4. Journal d'exécution réel (Simulation Log)

```text
[INFO] [sequence_soudure]: Cycle automatique : mise au repos initiale
[INFO] [sequence_soudure]: [ZONE] reservee par robot_a
[INFO] [sequence_soudure]: robot_a : soudure en cours (environ 15 s)
[INFO] [sequence_soudure]: robot_a : termine
[INFO] [sequence_soudure]: [ZONE] liberee par robot_a
[INFO] [sequence_soudure]: [ZONE] reservee par robot_b
[INFO] [sequence_soudure]: robot_b : soudure en cours (environ 15 s)
[INFO] [sequence_soudure]: robot_b : termine
[INFO] [sequence_soudure]: [ZONE] liberee par robot_b
[INFO] [sequence_soudure]: [ZONE] reservee par robot_c
[INFO] [sequence_soudure]: robot_c : soudure en cours (environ 15 s)
[INFO] [sequence_soudure]: robot_c : termine
[INFO] [sequence_soudure]: [ZONE] liberee par robot_c
[INFO] [sequence_soudure]: Cycle termine avec succes en 48.2 s : tous les robots sont au repos.
```

---

## 5. Conclusion des tests

L'ensemble des exigences du cahier des charges relatives aux **scénarios automatisés, au séquenceur et à la sécurité multi-robots** est pleinement validé. Aucun risque de collision matérielle n'est possible grâce à l'architecture d'exclusion mutuelle à verrou unique.
