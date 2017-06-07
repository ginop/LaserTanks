import unittest
import numpy as np
from Tank import Tank


class SimplisticTest(unittest.TestCase):

    def test_create_tank(self):
        """Should be able to define position and orientation as list or tuple
        or array and all numeric types should be converted to floats."""
        for n in [0, 0.]:
            for m in [[n, n], (n, n), np.array([n, n])]:
                msg = 'Failed to properly interpret a {} of {}s.'.format(type(m), type(n))
                T = Tank('LaserTankController', (0, 0, 0), m, m)
                self.assertTrue(np.all(T.position == np.array([0., 0.])), msg)
                self.assertTrue(np.all(T.orientation == np.array([0., 0.])), msg)

    def test_detect_hit(self):
        """Test laser hit detection"""
        T = Tank('LaserTankController', (0, 0, 0), (0, 0), (0, 0))
        # Override the tank shape to make testing easier
        T.body_length = 2.
        T.body_width = 2.
        # Laser defined as tuple of origin x and y and angle
        # A hit returns the distance to the impact, a miss returns None

        def hit_test(laser, output):
            if output is None:
                self.assertEqual(T.detect_hit(laser), None)
            else:
                self.assertTrue(abs(T.detect_hit(laser) - output) < 1e-12,
                                "{} != {}".format(T.detect_hit(laser), output))

        # To the right pointed right, a miss
        hit_test((2, 0, 0), None)
        hit_test((2, 0, 360), None)
        hit_test((2, 0, -360), None)
        # To the right pointed left, a hit
        hit_test((2, 0, 180), 1)
        # To the left pointed left, a miss
        hit_test((-2, 0, 180), None)
        # To the left pointed right, a hit
        hit_test((-2, 0, 0), 1)
        # Above pointed up, a miss
        hit_test((0, 2, 90), None)
        # Above pointed down, a hit
        hit_test((0, 2, -90), 1)
        # Below pointed down, a miss
        hit_test((0, -2, -90), None)
        # Below pointed up, a hit
        hit_test((0, -2, 90), 1)

        # Corner shot from NE
        hit_test((2, 2, 45), None)
        hit_test((2, 2, -135), np.sqrt(2))
        # Corner shot from SW
        hit_test((-2, -2, -45), None)
        hit_test((-2, -2, 45), np.sqrt(2))

        # Glancing blow
        hit_test((1, 2, -90), 1)
        hit_test((2, 1, 180), 1)
        hit_test((-1, -2, 90), 1)
        hit_test((-2, -1, 0), 1)

        # Angled impact (not 0 or 90 degrees)
        T.orientation[0] = 45.0
        hit_test((2, 2, -135), 2*np.sqrt(2) - 1)
        T.orientation[0] = 45.0
        hit_test((-2, -2, 45), 2*np.sqrt(2) - 1)
        # With target not on origin
        T.position = np.array([1/2, -1/2])
        T.orientation[0] = 45.0
        hit_test((2, 2, -135), 2*np.sqrt(2) - 1)
        T.orientation[0] = 45.0
        hit_test((-2, -2, 45), 2*np.sqrt(2) - 1)


if __name__ == '__main__':
    unittest.main()
