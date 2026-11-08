#!/usr/bin/env python3
"""Sequence automatique de soudage fictif (AUTOWELD-X).

Equipe AGROSMART-X - EIC 3.0
Responsable : NAIVELANDE Fortunat (Scenarios automatises et sequenceur)

Usage :
    python3 sequence_soudure.py cycle      # les 3 robots soudent l'un apres l'autre (defaut)
    python3 sequence_soudure.py repos      # les 3 robots reviennent a la verticale, d'un coup
    python3 sequence_soudure.py approche   # les 3 robots se placent en approche, d'un coup
    python3 sequence_soudure.py urgence    # arret d'urgence et repli de securite (E-STOP)
"""
import sys
import time

try:
    import rclpy
    from builtin_interfaces.msg import Duration
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    RCLPY_AVAILABLE = True
except ImportError:
    RCLPY_AVAILABLE = False
    Node = object

# Verification des trajectoires avant envoi (module de Hervens PIERRE)
try:
    from verification import verifier_trajectoire
except ImportError:
    from autoweld_x_control.verification import verifier_trajectoire

# Angle de la base de chaque robot pour faire face a la piece (rad)
ROBOTS = {'robot_a': 0.0, 'robot_b': 1.5708, 'robot_c': -1.5708}
ORDRE = ['robot_a', 'robot_b', 'robot_c']

# (epaule, coude) en radians, calcules pour la geometrie de la scene
Q_APPROCHE = (0.240, 1.863)   # torche en retrait, au-dessus de la piece
Q_SOUDURE = (0.634, 1.555)    # torche a environ 3 cm de la face a souder
DECALAGE = 0.09               # rotation de la base pendant le cordon (rad)
TOLERANCE = 0.05              # ecart accepte a l'arrivee (rad)


def duree(secondes):
    """Convertit un nombre flottant de secondes en structure Duration ROS 2."""
    if RCLPY_AVAILABLE:
        d = Duration()
        d.sec = int(secondes)
        d.nanosec = int((secondes - int(secondes)) * 1e9)
        return d
    return {'sec': int(secondes), 'nanosec': int((secondes - int(secondes)) * 1e9)}


class ZonePartagee:
    """Verrou d'exclusion mutuelle : un seul robot a la fois dans la zone critique."""

    def __init__(self, logger=None):
        self.proprietaire = None
        self.logger = logger

    def reserver(self, robot):
        if self.proprietaire is not None:
            raise RuntimeError(f'Zone deja occupee par {self.proprietaire}')
        self.proprietaire = robot
        if self.logger:
            self.logger.info(f'[ZONE] reservee par {robot}')

    def liberer(self, robot):
        if self.proprietaire != robot:
            raise RuntimeError(f'{robot} ne possede pas la zone')
        self.proprietaire = None
        if self.logger:
            self.logger.info(f'[ZONE] liberee par {robot}')


class CelluleSoudure(Node):
    def __init__(self):
        super().__init__(
            'sequence_soudure',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.pubs = {
            r: self.create_publisher(
                JointTrajectory, f'/{r}_controller/joint_trajectory', 10)
            for r in ROBOTS}
        self.etat = {}
        self.create_subscription(JointState, '/joint_states', self._recoit_etat, 10)

    def _recoit_etat(self, msg):
        for nom, pos in zip(msg.name, msg.position):
            self.etat[nom] = pos

    def attendre_controleurs(self, delai=60.0):
        debut = time.time()
        while rclpy.ok():
            if all(p.get_subscription_count() > 0 for p in self.pubs.values()):
                return
            if time.time() - debut > delai:
                raise RuntimeError(
                    'Controleurs introuvables : la simulation est-elle lancee ?')
            rclpy.spin_once(self, timeout_sec=0.2)

    def attendre(self, secondes):
        """Attend `secondes` de temps de simulation (repli sur l'horloge reelle
        si l'horloge de simulation n'avance pas)."""
        debut_sim = self.get_clock().now()
        debut_mur = time.time()
        while rclpy.ok():
            ecoule_sim = (self.get_clock().now() - debut_sim).nanoseconds * 1e-9
            ecoule_mur = time.time() - debut_mur
            horloge_morte = ecoule_mur > 5.0 and ecoule_sim < 0.01
            if horloge_morte and ecoule_mur >= secondes:
                return
            if not horloge_morte and ecoule_sim >= secondes:
                return
            rclpy.spin_once(self, timeout_sec=0.05)

    def envoyer(self, robot, points, verifier=True):
        # Controle avant envoi. Desactive pour l'arret d'urgence : le repli de
        # securite ne doit jamais etre bloque par une verification.
        if verifier:
            depart = [self.etat.get(f'{robot}_joint_{i}') for i in (1, 2, 3)]
            if None in depart:
                depart = None
            erreurs = verifier_trajectoire(points, depart=depart)
            if erreurs:
                for e in erreurs:
                    self.get_logger().error(f'[VERIF] {robot} : {e}')
                raise RuntimeError(f'Trajectoire refusee pour {robot}')
            self.get_logger().info(
                f'[VERIF] {robot} : trajectoire valide ({len(points)} point(s))')
        msg = JointTrajectory()
        msg.joint_names = [f'{robot}_joint_{i}' for i in (1, 2, 3)]
        for positions, t in points:
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in positions]
            p.time_from_start = duree(t)
            msg.points.append(p)
        self.pubs[robot].publish(msg)

    def verifier(self, robot, cible):
        rclpy.spin_once(self, timeout_sec=0.1)
        ecarts = [abs(self.etat.get(f'{robot}_joint_{i + 1}', 99.0) - cible[i])
                  for i in range(3)]
        if max(ecarts) > TOLERANCE:
            self.get_logger().warn(
                f'{robot} : position finale a {max(ecarts):.2f} rad de la cible')
            return False
        return True

    def tous(self, pose):
        """Met les 3 robots en position en meme temps."""
        for robot, q1 in ROBOTS.items():
            q = [q1, 0.0, 0.0] if pose == 'repos' else [q1, *Q_APPROCHE]
            self.envoyer(robot, [(q, 4.0)])
        self.attendre(5.0)

    def arret_urgence(self):
        """Securite simulee (E-STOP) : repli rapide de tous les bras au repos."""
        self.get_logger().warn('[SECURITE] Arret d\'urgence active : retour immediat au repos.')
        for robot, q1 in ROBOTS.items():
            self.envoyer(robot, [([q1, 0.0, 0.0], 2.0)], verifier=False)
        self.attendre(2.5)

    def cycle(self, robot):
        """Un cycle de soudure complet pour un robot, puis retour au repos."""
        q1 = ROBOTS[robot]
        a2, a3 = Q_APPROCHE
        s2, s3 = Q_SOUDURE
        points = [
            ([q1, a2, a3], 4.0),                 # approche
            ([q1 - DECALAGE, s2, s3], 6.0),      # debut du cordon
            ([q1 + DECALAGE, s2, s3], 9.0),      # fin du cordon
            ([q1, a2, a3], 11.0),                # retrait
            ([q1, 0.0, 0.0], 15.0),              # retour au repos
        ]
        self.get_logger().info(f'{robot} : soudure en cours (environ 15 s)')
        self.envoyer(robot, points)
        self.attendre(16.0)
        ok = self.verifier(robot, [q1, 0.0, 0.0])
        self.get_logger().info(f'{robot} : termine' + ('' if ok else ' (avec ecart)'))


def main():
    if not RCLPY_AVAILABLE:
        print("Erreur : rclpy non disponible. Veuillez executer ce script dans un environnement ROS 2.")
        sys.exit(1)

    mode = sys.argv[1] if len(sys.argv) > 1 else 'cycle'
    rclpy.init()
    noeud = CelluleSoudure()
    try:
        noeud.attendre_controleurs()
        if mode in ('repos', 'approche'):
            noeud.get_logger().info(f'Les 3 robots passent en position : {mode}')
            noeud.tous(mode)
        elif mode in ('urgence', 'estop'):
            noeud.arret_urgence()
        else:
            zone = ZonePartagee(noeud.get_logger())
            t_debut = time.time()
            noeud.get_logger().info('Cycle automatique : mise au repos initiale')
            noeud.tous('repos')
            for robot in ORDRE:
                zone.reserver(robot)
                noeud.cycle(robot)
                zone.liberer(robot)
            t_total = time.time() - t_debut
            noeud.get_logger().info(
                f'Cycle termine avec succes en {t_total:.1f} s : tous les robots sont au repos.')
    except KeyboardInterrupt:
        noeud.get_logger().warn('Interrompu par l\'utilisateur : declenchement du repli de securite')
        try:
            noeud.arret_urgence()
        except Exception:
            pass
    finally:
        noeud.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
