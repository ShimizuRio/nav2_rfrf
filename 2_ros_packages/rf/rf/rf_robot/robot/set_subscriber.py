import os
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Point
from std_msgs.msg import Int8MultiArray, Int16MultiArray
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy


class set_subscriber:
    def __init__(self, robot):
        self.robot = robot
        # console
        tmp_name = f"/Robot{self.robot.rid}/console2robot"
        self.target_floor = self.robot.create_subscription(  # [target_floor, target_id]
            Int8MultiArray,
            tmp_name,
            self.robot.console_listener_callback,
            1,
        )

        # estimator
        # self.pose = self.robot.create_subscription(
        #     PoseWithCovarianceStamped,
        #     "/pose",  # published by slam_toolbox
        #     self.robot.estimator.set_pose_callback,
        #     qos_profile_sensor_data,
        # )

        # rover
        self.rover_odo = self.robot.create_subscription(
            Twist,
            "/rover_odo",
            self.robot.rover.odometry_callback,
            qos_profile_sensor_data,
        )
        self.rover_sensor = self.robot.create_subscription(
            Int16MultiArray,
            "/rover_sensor",
            self.robot.rover.bumper_callback,
            qos_profile_sensor_data,
        )

        # rplidar
        self.lidarF = self.robot.create_subscription(
            LaserScan,
            #            "/" + os.environ["HOSTNAME"] + "/front_rplidar_node/scan_front",
            "/Drfrobot1/front_rplidar_node/scan_front",
            self.robot.rplidar.lidarF_callback,
            qos_profile_sensor_data,
        )
        self.lidarR = self.robot.create_subscription(
            LaserScan,
            #            "/" + os.environ["HOSTNAME"] + "/back_rplidar_node/scan_back",
            "/Drfrobot1/back_rplidar_node/scan_back",
            self.robot.rplidar.lidarB_callback,
            qos_profile_sensor_data,
        )

        # TO MATLAB
        # qos_profile = QoSProfile(depth=10)
        # qos_profile.reliability = ReliabilityPolicy.RELIABLE
        # self.pose_matlab = self.robot.create_subscription(
        #     Point,
        #     "/robot_state",
        #     self.robot.estimator.set_pose_callback_matlab,
        #     qos_profile,
        # )

        def destroy_subscription(self):
            self.robot.destroy_subscription(self.target_floor)
            self.robot.destroy_subscription(self.pose)
            self.robot.destroy_subscription(self.rover_odo)
            self.robot.destroy_subscription(self.rover_sensor)
            self.robot.destroy_subscription(self.lidarF)
            self.robot.destroy_subscription(self.lidarR)
            # TO MATLAB
            # self.robot.destroy_subscription(self.pose_matlab)
