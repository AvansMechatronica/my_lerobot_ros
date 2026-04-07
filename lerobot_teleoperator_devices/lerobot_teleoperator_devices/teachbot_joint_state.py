from typing import Any
from lerobot.teleoperators import Teleoperator

class TeachbotJointStateTeleop(Teleoperator):
    """
    Teleop class to subscribe to /teachbot/joint_states and relay joint states.
    """
    config_class = None  # Set to a config class if you want to support configuration
    name = "teachbot_joint_state"

    def __init__(self, config: Any = None):
        super().__init__(config)
        self.config = config
        self.latest_joint_state = None
        self._setup_ros2_subscriber()

    def _setup_ros2_subscriber(self):
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
        except ImportError:
            raise RuntimeError("ROS2 (rclpy, sensor_msgs) must be installed in the environment.")

        class JointStateListener(Node):
            def __init__(self, outer):
                super().__init__('teachbot_joint_state_listener')
                self.outer = outer
                self.subscription = self.create_subscription(
                    JointState,
                    '/teachbot/joint_states',
                    self.listener_callback,
                    10
                )

            def listener_callback(self, msg):
                self.outer.latest_joint_state = msg

        self.rclpy = rclpy
        self.node = JointStateListener(self)
        self.rclpy.init()

    def get_action(self) -> dict[str, float]:
        """
        Returns only the joint values (positions) from the latest joint state received from /teachbot/joint_states.
        """
        if self.latest_joint_state is not None and hasattr(self.latest_joint_state, 'position') and hasattr(self.latest_joint_state, 'name'):
            return {f"{name}.pos": pos for name, pos in zip(self.latest_joint_state.name, self.latest_joint_state.position)}
        return None
