"""Coordinateur multi-robots : zone d'exclusion partagee entre les robots A et B."""
import math
import os
import threading
import time
from datetime import datetime

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Limites des articulations (copiees du xacro)
LIMITES = {1: (-3.14, 3.14), 2: (-1.57, 1.57), 3: (-2.0, 2.0)}

# Longueurs utiles pour estimer la position de la torche (m)
L_BRAS = 0.40
L_AVANT_BRAS = 0.35

# Zone d'exclusion partagee : |x| < ZONE_X, au milieu de la cellule
ZONE_X = 0.25

# Chaque etape : (nom, [joint_1, joint_2, joint_3], duree en secondes)
CYCLE_A = [
    ('depart',    [0.0, 0.0, 0.0], 3.0),
    ('approche',  [0.0, 0.6, 0.3], 3.0),
    ('travail_1', [0.0, 1.2, 0.4], 3.0),
    ('travail_2', [0.0, 1.0, 0.8], 3.0),
    ('retrait',   [0.0, 0.6, 0.3], 3.0),
    ('retour',    [0.0, 0.0, 0.0], 3.0),
]
# Le robot B fait le mouvement inverse (miroir) du robot A
CYCLE_B = [(nom, [j1, -j2, -j3], d) for nom, (j1, j2, j3), d in CYCLE_A]


class Coordinateur(Node):
    def __init__(self):
        super().__init__('coordinateur')
        self.zone = threading.Lock()
        self.proprietaire = None
        self.verrou_journal = threading.Lock()
        dossier = os.path.expanduser('~/autoweld_ws/journaux')
        os.makedirs(dossier, exist_ok=True)
        self.fichier = os.path.join(dossier, 'coordination.log')
        self.pubs = {
            r: self.create_publisher(
                JointTrajectory, f'/{r}_arm_controller/joint_trajectory', 10)
            for r in ('a', 'b')}

    def noter(self, message):
        """Affiche le message et l'ecrit dans le journal."""
        self.get_logger().info(message)
        heure = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        with self.verrou_journal:
            with open(self.fichier, 'a') as f:
                f.write(f'{heure} {message}\n')

    def attendre_controleurs(self, delai=30.0):
        debut = time.time()
        while time.time() - debut < delai:
            if all(p.get_subscription_count() > 0 for p in self.pubs.values()):
                return True
            time.sleep(0.5)
        return False

    @staticmethod
    def x_outil(base_x, pos):
        """Position x estimee de la torche (valable si joint_1 = 0)."""
        t2, t3 = pos[1], pos[2]
        return base_x + L_BRAS * math.sin(t2) + L_AVANT_BRAS * math.sin(t2 + t3)

    def limites_ok(self, r, nom, pos):
        for i, valeur in enumerate(pos, start=1):
            bas, haut = LIMITES[i]
            if not bas <= valeur <= haut:
                self.noter(f'[{r.upper()}] REFUS {nom}: joint_{i}={valeur} '
                           f'hors limites [{bas}, {haut}]')
                return False
        return True

    def envoyer(self, r, pos, duree):
        msg = JointTrajectory()
        msg.joint_names = [f'{r}_joint_{i}' for i in (1, 2, 3)]
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in pos]
        point.time_from_start = Duration(
            sec=int(duree), nanosec=int((duree % 1) * 1e9))
        msg.points = [point]
        self.pubs[r].publish(msg)
        time.sleep(duree + 0.5)

    def executer_cycle(self, r, base_x, cycle):
        R = r.upper()
        try:
            for nom, pos, duree in cycle:
                if not self.limites_ok(r, nom, pos):
                    self.noter(f'[{R}] cycle ARRETE (limites)')
                    return
                x = self.x_outil(base_x, pos)
                dans = abs(x) < ZONE_X

                if dans and self.proprietaire != r:
                    self.noter(f'[{R}] demande la zone pour {nom} '
                               f'(x_outil={x:+.2f} m)')
                    if not self.zone.acquire(blocking=False):
                        autre = (self.proprietaire or '?').upper()
                        self.noter(f'[{R}] ATTEND : zone occupee par {autre}')
                        self.zone.acquire()
                    self.proprietaire = r
                    self.noter(f'[{R}] ENTRE dans la zone')

                self.noter(f'[{R}] va vers {nom} (x_outil={x:+.2f} m)')
                self.envoyer(r, pos, duree)
                self.noter(f'[{R}] a atteint {nom}')

                if not dans and self.proprietaire == r:
                    self.proprietaire = None
                    self.zone.release()
                    self.noter(f'[{R}] LIBERE la zone')
            self.noter(f'[{R}] cycle termine')
        finally:
            if self.proprietaire == r:
                self.proprietaire = None
                self.zone.release()


def main():
    rclpy.init()
    noeud = Coordinateur()
    if not noeud.attendre_controleurs():
        noeud.noter('ERREUR : controleurs introuvables. La simulation tourne-t-elle ?')
        noeud.destroy_node()
        rclpy.shutdown()
        return
    noeud.noter('=== Debut du test de coordination A/B ===')
    fils = [
        threading.Thread(target=noeud.executer_cycle, args=('a', -0.8, CYCLE_A)),
        threading.Thread(target=noeud.executer_cycle, args=('b', 0.8, CYCLE_B)),
    ]
    for f in fils:
        f.start()
    for f in fils:
        f.join()
    noeud.noter('=== Fin du test ===')
    noeud.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
