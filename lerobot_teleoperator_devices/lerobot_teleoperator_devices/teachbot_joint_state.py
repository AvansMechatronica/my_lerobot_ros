
from typing import Any, Optional
from lerobot.teleoperators import Teleoperator
from .config_teachbot_joint_state import TeachbotTeleopConfig


class TeachbotJointStateTeleop(Teleoperator):
    """
    Teleoperator device for relaying joint states from /teachbot/joint_states via ROS2.
    Implements LeRobot teleoperator conventions for integration and discoverability.
    """
    name = "teachbot_joint_state"
    config_class = TeachbotTeleopConfig


    def __init__(self, config: TeachbotTeleopConfig):
        """
        Initialize the teleoperator with the given configuration.
        """
        super().__init__(config)
        self.config = config
        self.latest_joint_state: Optional[Any] = None
        self.rclpy = None
        self.node = None
        self._ros2_initialized = False

    def _setup_ros2_subscriber(self) -> None:
        """
        Set up the ROS2 subscriber for /teachbot/joint_states.
        """
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
        self._ros2_initialized = True

    def get_action(self) -> Optional[dict[str, float]]:
        """
        Return the latest joint positions as a dict, or None if no data is available.
        Keys are joint names with '.pos' suffix, values are positions.
        """
        if self.latest_joint_state is not None and hasattr(self.latest_joint_state, 'position') and hasattr(self.latest_joint_state, 'name'):
            return {f"{name}.pos": pos for name, pos in zip(self.latest_joint_state.name, self.latest_joint_state.position)}
        return None

    def connect(self) -> None:
        """
        Establish connection to the teleoperator hardware (ROS2 node).
        """
        if not self._ros2_initialized:
            self._setup_ros2_subscriber()

    def disconnect(self) -> None:
        """
        Gracefully disconnect from the teleoperator hardware (ROS2 node).
        """
        if self.rclpy and self.node:
            self.rclpy.shutdown()
            self.node = None
            self._ros2_initialized = False
