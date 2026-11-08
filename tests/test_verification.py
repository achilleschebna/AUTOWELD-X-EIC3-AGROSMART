#!/usr/bin/env python3
"""Tests du module de verification des trajectoires (AUTOWELD-X).

Responsable : PIERRE Hervens (Programmation Python / logique de controle)
Execution (sans simulateur) : python3 -m unittest tests/test_verification.py -v
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', 'src', 'autoweld_x_control', 'autoweld_x_control')))

from verification import verifier_pose, verifier_trajectoire
from sequence_soudure import ROBOTS, Q_APPROCHE, Q_SOUDURE, DECALAGE


def points_du_cycle(q1):
    """Reproduit les 5 points de CelluleSoudure.cycle() pour un robot."""
    a2, a3 = Q_APPROCHE
    s2, s3 = Q_SOUDURE
    return [
        ([q1, a2, a3], 4.0),
        ([q1 - DECALAGE, s2, s3], 6.0),
        ([q1 + DECALAGE, s2, s3], 9.0),
        ([q1, a2, a3], 11.0),
        ([q1, 0.0, 0.0], 15.0),
    ]


class TestPose(unittest.TestCase):
    def test_pose_valide(self):
        self.assertEqual(verifier_pose([0.0, 0.5, 1.0]), [])

    def test_pose_hors_limites(self):
        erreurs = verifier_pose([0.0, 2.0, 0.0])
        self.assertEqual(len(erreurs), 1)
        self.assertIn('joint_2', erreurs[0])

    def test_mauvais_nombre_articulations(self):
        self.assertEqual(len(verifier_pose([0.0, 0.0])), 1)


class TestTrajectoire(unittest.TestCase):
    def test_cycle_reel_valide_pour_chaque_robot(self):
        for robot, q1 in ROBOTS.items():
            with self.subTest(robot=robot):
                erreurs = verifier_trajectoire(points_du_cycle(q1), depart=[q1, 0.0, 0.0])
                self.assertEqual(erreurs, [])

    def test_trajectoire_vide(self):
        self.assertEqual(verifier_trajectoire([]), ['trajectoire vide'])

    def test_temps_non_croissant(self):
        points = [([0.0, 0.0, 0.0], 4.0), ([0.0, 0.1, 0.1], 3.0)]
        erreurs = verifier_trajectoire(points)
        self.assertTrue(any('non croissant' in e for e in erreurs))

    def test_vitesse_excessive(self):
        points = [([0.0, 0.0, 1.0], 0.1)]
        erreurs = verifier_trajectoire(points, depart=[0.0, 0.0, 0.0])
        self.assertTrue(any('vitesse' in e for e in erreurs))

    def test_pose_hors_limites_dans_trajectoire(self):
        points = [([0.0, 0.0, 2.5], 5.0)]
        erreurs = verifier_trajectoire(points)
        self.assertTrue(any('joint_3' in e for e in erreurs))


if __name__ == '__main__':
    unittest.main()
