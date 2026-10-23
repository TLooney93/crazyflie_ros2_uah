"""Translate validated wand gestures into guarded drone ROS commands."""

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from cf_control.config import WAND_GESTURE_TOPIC, WAND_TARGET_STEP_M


class WandCoordinator(Node):
    """Bridge device-neutral wand gestures to one selected vehicle."""

    def __init__(self):
        super().__init__('wand_coordinator')
        self.declare_parameter('drone_id', 'cf1')
        self.drone_id = self.get_parameter('drone_id').value
        self.declare_parameter('movement_step_m', WAND_TARGET_STEP_M)
        self.movement_step_m = float(
            self.get_parameter('movement_step_m').value
        )
        self.enabled = False
        self.position = None
        self.pending_service = None

        self.gesture_sub = self.create_subscription(
            String, WAND_GESTURE_TOPIC, self.gesture_callback, 10,
        )
        self.position_sub = self.create_subscription(
            Point,
            f'/{self.drone_id}/position',
            self.position_callback,
            10,
        )
        self.target_pub = self.create_publisher(
            Point, f'/{self.drone_id}/target', 10,
        )
        self.takeoff_client = self.create_client(
            Trigger, f'/{self.drone_id}/takeoff',
        )
        self.land_client = self.create_client(
            Trigger, f'/{self.drone_id}/land',
        )
        self.stop_client = self.create_client(
            Trigger, f'/{self.drone_id}/emergency_stop',
        )
        self.get_logger().info(
            f'Wand coordinator ready for {self.drone_id}; disabled by default'
        )

    def position_callback(self, msg):
        """Store the latest selected-drone position."""
        position = (msg.x, msg.y, msg.z)
        if all(math.isfinite(value) for value in position):
            self.position = position

    def _call(self, client, label, priority=False):
        if self.pending_service is not None and not priority:
            self.get_logger().warning('Ignoring gesture while service is pending')
            return
        if not client.service_is_ready():
            self.get_logger().error(f'{label} service is unavailable')
            return
        future = client.call_async(Trigger.Request())
        self.pending_service = future

        def done(completed):
            if self.pending_service is completed:
                self.pending_service = None
            try:
                result = completed.result()
                log = self.get_logger().info if result.success else self.get_logger().error
                log(f'{label}: {result.message}')
            except Exception as error:  # ROS future exceptions vary by backend.
                self.get_logger().error(f'{label} failed: {error}')

        future.add_done_callback(done)

    def gesture_callback(self, msg):
        """Apply one normalized wand gesture."""
        gesture = msg.data.strip().lower()
        if gesture == 'enable':
            self.enabled = True
            self.get_logger().warning('Wand command output ENABLED')
            return
        if gesture == 'disable':
            self.enabled = False
            self.get_logger().info('Wand command output disabled')
            return
        if gesture == 'land':
            self._call(self.land_client, 'land')
            return
        if gesture == 'emergency_stop':
            self.enabled = False
            self._call(self.stop_client, 'emergency stop', priority=True)
            return
        if gesture == 'hold':
            return
        if not self.enabled:
            self.get_logger().warning(
                f'Ignoring {gesture}: wand coordinator is disabled'
            )
            return
        if gesture == 'takeoff':
            self._call(self.takeoff_client, 'takeoff')
            return
        offsets = {
            'move_forward': (self.movement_step_m, 0.0, 0.0),
            'move_backward': (-self.movement_step_m, 0.0, 0.0),
            'move_left': (0.0, self.movement_step_m, 0.0),
            'move_right': (0.0, -self.movement_step_m, 0.0),
        }
        if gesture not in offsets or self.position is None:
            self.get_logger().warning(f'Cannot apply gesture: {gesture}')
            return
        dx, dy, dz = offsets[gesture]
        target = Point()
        target.x = self.position[0] + dx
        target.y = self.position[1] + dy
        target.z = self.position[2] + dz
        self.target_pub.publish(target)


def main(args=None):
    rclpy.init(args=args)
    node = WandCoordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
