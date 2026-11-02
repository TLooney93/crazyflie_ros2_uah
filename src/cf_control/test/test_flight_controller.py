import unittest

from cf_control.config import (
    TAKEOFF_BOOST_END_HEIGHT_M,
    TAKEOFF_BOOST_M_S,
)
from cf_control.flight_controller import (
    FlightController,
    ground_effect_takeoff_boost,
    limit_horizontal_velocity,
)


class FlightControllerTest(unittest.TestCase):

    def test_ground_effect_boost_only_applies_below_cutoff(self):
        self.assertEqual(
            ground_effect_takeoff_boost(
                0.05,
                TAKEOFF_BOOST_END_HEIGHT_M,
                TAKEOFF_BOOST_M_S,
            ),
            0.12,
        )
        self.assertEqual(
            ground_effect_takeoff_boost(
                TAKEOFF_BOOST_END_HEIGHT_M,
                TAKEOFF_BOOST_END_HEIGHT_M,
                TAKEOFF_BOOST_M_S,
            ),
            0.0,
        )

    def test_no_target_produces_zero_velocity(self):
        controller = FlightController(kp=0.5, max_velocity=0.3)

        self.assertEqual(
            controller.update((1.0, 2.0, 3.0)),
            (0.0, 0.0, 0.0),
        )

    def test_horizontal_velocity_limit_preserves_direction(self):
        vx, vy = limit_horizontal_velocity(0.3, 0.4, 0.1)

        self.assertAlmostEqual(vx, 0.06)
        self.assertAlmostEqual(vy, 0.08)

    def test_horizontal_velocity_below_limit_is_unchanged(self):
        self.assertEqual(
            limit_horizontal_velocity(0.03, -0.04, 0.1),
            (0.03, -0.04),
        )

    def test_position_error_produces_proportional_velocity(self):
        controller = FlightController(kp=0.5, max_velocity=0.3)
        controller.set_target(0.2, -0.4, 0.1)

        self.assertEqual(
            controller.update((0.0, 0.0, 0.0)),
            (0.1, -0.2, 0.05),
        )

    def test_velocity_is_limited_on_each_axis(self):
        controller = FlightController(kp=1.0, max_velocity=0.3)
        controller.set_target(2.0, -2.0, 0.3)

        self.assertEqual(
            controller.update((0.0, 0.0, 0.0)),
            (0.3, -0.3, 0.3),
        )

    def test_integral_correction_is_bounded_and_resets_for_new_target(self):
        controller = FlightController(
            kp=0.5,
            ki=0.2,
            max_velocity=0.3,
            integral_max_velocity=0.06,
        )
        controller.set_target(0.0, 0.0, 1.0)

        for _ in range(100):
            velocity = controller.update((0.0, 0.0, 0.9), dt_s=0.05)

        self.assertAlmostEqual(velocity[2], 0.11)
        controller.set_target(0.0, 0.0, 0.9)
        self.assertEqual(
            controller.update((0.0, 0.0, 0.9), dt_s=0.05),
            (0.0, 0.0, 0.0),
        )

    def test_horizontal_target_change_preserves_vertical_integral(self):
        controller = FlightController(
            kp=0.5,
            ki=0.2,
            max_velocity=0.3,
            integral_max_velocity=0.06,
        )
        controller.set_target(0.0, 0.0, 1.0)
        for _ in range(20):
            controller.update((0.0, 0.0, 0.9), dt_s=0.05)

        vertical_integral = controller.integral_error[2]
        controller.set_target(0.1, -0.1, 1.0)

        self.assertEqual(controller.integral_error[0], 0.0)
        self.assertEqual(controller.integral_error[1], 0.0)
        self.assertEqual(controller.integral_error[2], vertical_integral)


if __name__ == '__main__':
    unittest.main()
