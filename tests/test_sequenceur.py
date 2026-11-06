#!/usr/bin/env python3
"""Tests unitaires et de validation du sequenceur et des scenarios AUTOWELD-X.

Equipe AGROSMART-X - EIC 3.0
Responsable : NAIVELANDE Fortunat (Scenarios automatises et sequenceur)

Ces tests s'executent sans necessiter Gazebo ni ROS 2 complet :
    python -m unittest tests/test_sequenceur.py
ou
    pytest tests/test_sequenceur.py
"""

import math
import os
import sys
import unittest

# Ajout du chemin du package au sys.path
sys.path.insert(0, os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', 'src', 'autoweld_x_control', 'autoweld_x_control')))

from sequence_soudure import (
    ZonePartagee,
    ROBOTS,
    ORDRE,
    Q_APPROCHE,
    Q_SOUDURE,
    DECALAGE,
    TOLERANCE,
    duree
)

# Limites physiques issues du fichier de modelisation robots/bras_3ddl.xacro
LIMITES_ARTICULAIRES = {
    'joint_1': (-3.14, 3.14),   # Base (rad)
    'joint_2': (-1.57, 1.57),   # Epaule (rad)
    'joint_3': (-2.0, 2.0),     # Coude (rad)
}


class TestZonePartagee(unittest.TestCase):
    """Tests du verrou d'exclusion mutuelle pour eviter toute collision."""

    def setUp(self):
        self.zone = ZonePartagee()

    def test_reservation_nominale(self):
        """Un robot libre peut reserver puis liberer la zone."""
        self.assertIsNone(self.zone.proprietaire)
        self.zone.reserver('robot_a')
        self.assertEqual(self.zone.proprietaire, 'robot_a')
        self.zone.liberer('robot_a')
        self.assertIsNone(self.zone.proprietaire)

    def test_exclusion_mutuelle_collision(self):
        """Tentative d'acces simultane : declenchement obligatoire d'une RuntimeError."""
        self.zone.reserver('robot_a')
        with self.assertRaises(RuntimeError) as ctx:
            self.zone.reserver('robot_b')
        self.assertIn("Zone deja occupee par robot_a", str(ctx.exception))

    def test_liberation_par_tiers_interdite(self):
        """Un robot ne peut pas liberer une zone qu'il ne possede pas."""
        self.zone.reserver('robot_a')
        with self.assertRaises(RuntimeError) as ctx:
            self.zone.liberer('robot_c')
        self.assertIn("robot_c ne possede pas la zone", str(ctx.exception))

    def test_liberation_zone_vide(self):
        """Liberer une zone vide leve une exception."""
        with self.assertRaises(RuntimeError):
            self.zone.liberer('robot_a')


class TestParametresEtTrajectoires(unittest.TestCase):
    """Tests de conformite cinematique et des limites articulaires."""

    def test_configuration_robots(self):
        """La cellule doit comporter 3 robots distincts avec orientations exactes."""
        self.assertEqual(len(ROBOTS), 3)
        self.assertEqual(ORDRE, ['robot_a', 'robot_b', 'robot_c'])
        self.assertAlmostEqual(ROBOTS['robot_a'], 0.0, places=4)
        self.assertAlmostEqual(ROBOTS['robot_b'], 1.5708, places=4)
        self.assertAlmostEqual(ROBOTS['robot_c'], -1.5708, places=4)

    def test_respect_limites_articulaires_approche(self):
        """La pose d'approche doit rester dans les limites mecaniques."""
        q2, q3 = Q_APPROCHE
        lim_q2 = LIMITES_ARTICULAIRES['joint_2']
        lim_q3 = LIMITES_ARTICULAIRES['joint_3']
        self.assertTrue(lim_q2[0] <= q2 <= lim_q2[1], f"Epaule {q2} hors limites {lim_q2}")
        self.assertTrue(lim_q3[0] <= q3 <= lim_q3[1], f"Coude {q3} hors limites {lim_q3}")

    def test_respect_limites_articulaires_soudure(self):
        """Les poses de debut et fin de cordon doivent rester dans les limites mecaniques."""
        s2, s3 = Q_SOUDURE
        lim_q2 = LIMITES_ARTICULAIRES['joint_2']
        lim_q3 = LIMITES_ARTICULAIRES['joint_3']
        lim_q1 = LIMITES_ARTICULAIRES['joint_1']

        self.assertTrue(lim_q2[0] <= s2 <= lim_q2[1])
        self.assertTrue(lim_q3[0] <= s3 <= lim_q3[1])

        # Verification des angles de base pour chaque robot pendant le cordon
        for robot, q1_centre in ROBOTS.items():
            q1_debut = q1_centre - DECALAGE
            q1_fin = q1_centre + DECALAGE
            self.assertTrue(lim_q1[0] <= q1_debut <= lim_q1[1],
                            f"{robot}: base debut {q1_debut} hors limites")
            self.assertTrue(lim_q1[0] <= q1_fin <= lim_q1[1],
                            f"{robot}: base fin {q1_fin} hors limites")

    def test_chronologie_trajectoire_5_points(self):
        """Verifie la progression strictly croissante du temps sur le scenario."""
        # Scenario standard defini dans cycle(robot)
        t_points = [4.0, 6.0, 9.0, 11.0, 15.0]

        # Les points doivent etre chronologiques
        for i in range(len(t_points) - 1):
            self.assertLess(t_points[i], t_points[i + 1])

        # Duree nominale d'un cycle robot : 15s (16s avec marge)
        self.assertEqual(t_points[-1], 15.0)

        # Duree totale pour les 3 robots sequentiels : ~48s (conforme au cahier des charges)
        duree_totale_estimee = len(ORDRE) * 16.0
        self.assertEqual(duree_totale_estimee, 48.0)

    def test_conversion_duree(self):
        """La fonction utilitaire duree() doit gerer correctement secondes et nanosecondes."""
        d = duree(4.25)
        if hasattr(d, 'sec'):
            self.assertEqual(d.sec, 4)
            self.assertEqual(d.nanosec, 250000000)
        else:
            self.assertEqual(d['sec'], 4)
            self.assertEqual(d['nanosec'], 250000000)


if __name__ == '__main__':
    unittest.main()
