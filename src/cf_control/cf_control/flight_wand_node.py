"""Run the flight controller and physical button wand on one Crazyradio."""

import cflib.crtp
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from cf_control.crazyflie_node import CrazyflieNode
from cf_control.wand_button_node import WandButtonNode


def main(args=None):
    """Share cflib's in-process radio manager between cf1 and cf2."""
    rclpy.init(args=args)
    cflib.crtp.init_drivers()
    flight = None
    wand = None
    executor = MultiThreadedExecutor(num_threads=3)

    try:
        flight = CrazyflieNode()
        wand = WandButtonNode()
        flight.connect()
        wand.connect()
        executor.add_node(flight)
        executor.add_node(wand)
        flight.get_logger().info(
            'Combined cf1 + cf2 radio process ready'
        )
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if wand is not None:
            try:
                wand.disconnect()
            finally:
                wand.destroy_node()
        if flight is not None:
            try:
                flight.disconnect()
            finally:
                flight.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
