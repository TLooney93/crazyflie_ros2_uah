"""Bounded position controller used by hardware and simulation."""


class FlightController:
    """Bounded PI position controller shared by hardware and simulation."""

    def __init__(
        self,
        kp,
        max_velocity,
        ki=0.0,
        integral_max_velocity=0.0,
    ):
        """Create a controller with bounded proportional/integral output."""
        self.kp = kp
        self.ki = ki
        self.max_velocity = max_velocity
        self.integral_max_velocity = integral_max_velocity

        self.target = None
        self.integral_error = [0.0, 0.0, 0.0]

    def set_target(self, x, y, z):
        """Set an absolute position target and reset changed integrators."""
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
        """Remove the active target and accumulated integral error."""
        self.target = None
        self.integral_error = [0.0, 0.0, 0.0]

    def update(self, current_position, dt_s=0.0):
        """Return a bounded XYZ velocity for the current position."""
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
