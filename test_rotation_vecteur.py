import unittest

from trajectoir3 import rotationVecteur
from math import pi


class TestRotationVector(unittest.TestCase):
    def _check_vectors_equals(self, v1, v2):
        for i in range(3):
            self.assertEqual(v1[i], v2[i])

    def test_trivial1(self):
        v_in = [1, 0, 0]
        v_rot = [0.3, 0, 0] # angle quelconque

        v_out = rotationVecteur(v_in, v_rot)
        self._check_vectors_equals(v_in, v_out)

    def test_trivial2(self):
        v_in = [0, 1, 0]
        v_rot = [0, 0.8, 0] # angle quelconque

        v_out = rotationVecteur(v_in, v_rot)
        self._check_vectors_equals(v_in, v_out)

    def test_trivial3(self):
        v_in = [0, 0, 1]
        v_rot = [0, 0, 12.4] # angle quelconque

        v_out = rotationVecteur(v_in, v_rot)
        self._check_vectors_equals(v_in, v_out)

    def test_rotate1(self):
        """Tourner d'un angle pi/2 devrait amener (1,0,0) vers (0,0,-1)."""
        v_in = [1, 0, 0]
        v_rot = [0, pi/2, 0]

        v_out = rotationVecteur(v_in, v_rot)
        v_check = [0,0,-1]
        self._check_vectors_equals(v_in, v_out)
