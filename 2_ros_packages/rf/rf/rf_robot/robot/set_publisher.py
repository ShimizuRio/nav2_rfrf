from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Int16MultiArray
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy


class set_publisher:
    def __init__(self, robot):
        self.robot = robot
        # HB
        tmp_name = f"/Robot{self.robot.rid}/Heartbeat"
        self.hb = self.robot.create_publisher(
            String,
            tmp_name,
            qos_profile_sensor_data,
        )

        # rover
        self.input = self.robot.create_publisher(Twist, "/rover_twist", 1)

        # to slam_toolbox
        qos_profile = QoSProfile(depth=1)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        self.scan = self.robot.create_publisher(LaserScan, "/scan", qos_profile)

        # TO MATLAB
        qos_profile = QoSProfile(depth=10)
        self.ref = self.robot.create_publisher(
            Point, "/rf_reference_point", qos_profile
        )
