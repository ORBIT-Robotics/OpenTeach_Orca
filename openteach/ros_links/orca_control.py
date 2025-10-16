"""
orca_hand_link.py
-----------------
ROS 2 (other control files are ROS 1) bridge between OpenTeach and the ORCA Hand.

Publishes:
    /orca_hand/command        (std_msgs/Float64MultiArray)
Subscribes:
    /orca_hand/joint_states   (sensor_msgs/JointState)

Exposes:
    get_joint_angles()  → current joint positions (np.ndarray)
    send_joint_cmd(q)   → publish new joint command (np.ndarray)
"""

import numpy as np
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class OrcaHandLink(Node):
    """ROS 2 communication bridge for the ORCA Hand."""

    CMD_TOPIC = "/orca_hand/command"
    STATE_TOPIC = "/orca_hand/joint_states"

    def __init__(self, publish_rate_hz: float = 100.0):
        super().__init__("orca_hand_link")

        # Publisher (send commands to hand)
        self.pub = self.create_publisher(Float64MultiArray, self.CMD_TOPIC, 10)

        # Subscriber (receive joint states from hand)
        self.sub = self.create_subscription(
            JointState,
            self.STATE_TOPIC,
            self._state_callback,
            10
        )

        # Internal state buffer
        self._latest_state = np.zeros(15)
        self._lock = threading.Lock()

        # Optional debug heartbeat
        self.create_timer(1.0, lambda: self.get_logger().debug("orca_hand_link alive"))

        self._period = 1.0 / publish_rate_hz

    def _state_callback(self, msg: JointState) -> None:
        """Callback for joint state updates from the hand."""
        with self._lock:
            self._latest_state[:] = msg.position

    def get_joint_angles(self) -> np.ndarray:
        """Return the latest known joint angles (radians)."""
        with self._lock:
            return self._latest_state.copy()

    def send_joint_cmd(self, q_rad: np.ndarray) -> None:
        """Send a new joint command to the hand."""
        msg = Float64MultiArray(data=q_rad.tolist())
        self.pub.publish(msg)


if __name__ == "__main__":
    rclpy.init()
    node = OrcaHandLink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
