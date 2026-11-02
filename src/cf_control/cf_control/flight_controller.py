import math


def ground_effect_takeoff_boost(
    height_above_floor_m,
    boost_end_height_m,
    boost_m_s,
):
    """Return a bounded vertical boost while the vehicle is near the floor."""
    if height_above_floor_m < boost_end_height_m:
        return boost_m_s
    return 0.0


def limit_horizontal_velocity(vx, vy, max_velocity):
    """Limit horizontal speed while preserving the requested direction."""
    speed = math.hypot(vx, vy)
    if speed <= max_velocity or speed == 0.0:
        return vx, vy
    scale = max_velocity / speed
    return vx * scale, vy * scale


class FlightController:

    def __init__(
        self,
        kp,
        max_velocity,
        ki=0.0,
        integral_max_velocity=0.0,
    ):
        self.kp = kp
        self.ki = ki
        self.max_velocity = max_velocity
        self.integral_max_velocity = integral_max_velocity

        self.target = None
        self.integral_error = [0.0, 0.0, 0.0]

    def set_target(self, x, y, z):
        target = (x, y, z)
        if self.target is None:
            self.integral_error = [0.0, 0.0, 0.0]
        else:
            for index, (old_value, new_value) in enumerate(
                zip(self.target, target)
            ):
                if old_value != new_value:
                    self.integral_error[index] = 0.0
        self.target = target

    def clear_target(self):
        self.target = None
        self.integral_error = [0.0, 0.0, 0.0]

    def update(self, current_position, dt_s=0.0):

        if self.target is None:
            return 0.0, 0.0, 0.0

        x, y, z = current_position
        tx, ty, tz = self.target

        error_x = tx - x
        error_y = ty - y
        error_z = tz - z

        errors = (error_x, error_y, error_z)
        if self.ki > 0.0 and dt_s > 0.0:
            integral_error_limit = self.integral_max_velocity / self.ki
            for index, error in enumerate(errors):
                self.integral_error[index] += error * dt_s
                self.integral_error[index] = max(
                    -integral_error_limit,
                    min(self.integral_error[index], integral_error_limit),
                )

        velocities = [
            self.kp * error + self.ki * integral
            for error, integral in zip(errors, self.integral_error)
        ]

        vx, vy, vz = (self._limit(value) for value in velocities)

        return vx, vy, vz

    def _limit(self, value):

        if value > self.max_velocity:
            return self.max_velocity

        if value < -self.max_velocity:
            return -self.max_velocity

        return value
