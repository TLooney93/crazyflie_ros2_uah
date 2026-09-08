class FlightController:

    def __init__(self, kp, max_velocity):
        self.kp = kp
        self.max_velocity = max_velocity

        self.target = None

    def set_target(self, x, y, z):
        self.target = (x, y, z)

    def clear_target(self):
        self.target = None

    def update(self, current_position):

        if self.target is None:
            return 0.0, 0.0, 0.0

        x, y, z = current_position
        tx, ty, tz = self.target

        error_x = tx - x
        error_y = ty - y
        error_z = tz - z

        vx = self.kp * error_x
        vy = self.kp * error_y
        vz = self.kp * error_z

        vx = self._limit(vx)
        vy = self._limit(vy)
        vz = self._limit(vz)

        return vx, vy, vz

    def _limit(self, value):

        if value > self.max_velocity:
            return self.max_velocity

        if value < -self.max_velocity:
            return -self.max_velocity

        return value