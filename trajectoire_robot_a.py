#!/usr/bin/env python3
"""
Trajectoire de soudure du robot A (Gazebo Classic + ROS 2).

Cycle : home -> approche -> soudure le long du joint rouge (x=-0.30 m,
y de -0.15 a +0.15 m, 5 cm/s) -> retrait -> home, puis recommence.

Les angles des points cles ont ete calcules par cinematique inverse
(torche verticale, 12 mm au-dessus du joint, erreur < 0.2 mm).

Lancement (Gazebo doit deja tourner) :
    python3 trajectoire_robot_a.py
"""
import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINTS = [f"robot_a_joint_{i}" for i in range(1, 7)]
RATE = 50.0  # Hz, identique a <update_rate> du plugin dans le .world

# (nom, angles [rad] des 6 articulations, duree pour y arriver [s], mouvement lisse)
KEYFRAMES = [
    ("home", [0.0000, -0.2000, 1.0000, 0.0000, 0.9000, 0.0000], 0, True),
    ("approche", [-0.2448, 0.2022, -0.1017, -0.0001, 1.4694, -0.2446], 3.0, True),
    ("debut_soudure", [-0.2450, 0.2460, 0.0881, -0.0000, 1.2367, -0.2450], 1.5, True),
    ("soudure_1", [-0.1974, 0.2356, 0.1016, 0.0000, 1.2336, -0.1975], 0.6, False),
    ("soudure_2", [-0.1489, 0.2274, 0.1122, 0.0000, 1.2312, -0.1490], 0.6, False),
    ("soudure_3", [-0.0997, 0.2215, 0.1197, 0.0000, 1.2295, -0.0998], 0.6, False),
    ("soudure_4", [-0.0500, 0.2180, 0.1243, 0.0000, 1.2286, -0.0500], 0.6, False),
    ("soudure_5", [-0.0000, 0.2168, 0.1258, 0.0000, 1.2282, -0.0001], 0.6, False),
    ("soudure_6", [0.0499, 0.2180, 0.1243, 0.0000, 1.2285, 0.0499], 0.6, False),
    ("soudure_7", [0.0996, 0.2215, 0.1198, 0.0000, 1.2295, 0.0996], 0.6, False),
    ("soudure_8", [0.1488, 0.2274, 0.1122, 0.0000, 1.2312, 0.1488], 0.6, False),
    ("soudure_9", [0.1974, 0.2356, 0.1016, 0.0000, 1.2336, 0.1973], 0.6, False),
    ("soudure_10", [0.2449, 0.2460, 0.0881, 0.0000, 1.2367, 0.2449], 0.6, False),
    ("retrait", [0.2450, 0.2028, -0.1031, 0.0000, 1.4709, 0.2450], 1.5, True),
    ("home", [0.0000, -0.2000, 1.0000, 0.0000, 0.9000, 0.0000], 3.0, True),
]
PAUSE = 1.0  # s d'attente en position home entre deux cycles


def interpoler():
    """Echantillonne les points cles a RATE Hz."""
    points = [KEYFRAMES[0][1]]
    for (_, q0, _, _), (_, q1, duree, lisse) in zip(KEYFRAMES[:-1], KEYFRAMES[1:]):
        n = max(1, int(round(duree * RATE)))
        for k in range(1, n + 1):
            t = k / n
            if lisse:  # acceleration et freinage progressifs
                t = 0.5 - 0.5 * math.cos(math.pi * t)
            points.append([a + (b - a) * t for a, b in zip(q0, q1)])
    points += [KEYFRAMES[-1][1]] * int(PAUSE * RATE)
    return points


class TrajectoireRobotA(Node):
    def __init__(self):
        super().__init__("trajectoire_robot_a")
        self.pub = self.create_publisher(JointTrajectory, "/robot_a/set_joint_trajectory", 10)
        self.points = interpoler()
        self.duree_cycle = len(self.points) / RATE
        self.cycle = 0
        self.timer = self.create_timer(0.5, self.attendre_gazebo)

    def attendre_gazebo(self):
        if self.pub.get_subscription_count() == 0:
            self.get_logger().info("En attente du plugin Gazebo /robot_a/set_joint_trajectory ...")
            return
        self.timer.cancel()
        self.envoyer()
        self.timer = self.create_timer(self.duree_cycle + 0.2, self.envoyer)

    def envoyer(self):
        msg = JointTrajectory()
        msg.header.frame_id = "world"
        msg.joint_names = JOINTS
        for i, q in enumerate(self.points):
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in q]
            t = i / RATE
            p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            msg.points.append(p)
        self.pub.publish(msg)
        self.cycle += 1
        self.get_logger().info(
            f"Cycle {self.cycle} envoye : {len(msg.points)} points, {self.duree_cycle:.1f} s")


def main():
    rclpy.init()
    node = TrajectoireRobotA()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
