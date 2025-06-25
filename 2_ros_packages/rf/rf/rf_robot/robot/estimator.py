# from nav_msgs import OccupancyGrid
import rclpy
from time import time
import numpy as np
from tf2_ros import TransformListener, Buffer

# from geometry_msgs.msg import TransformStamped


class ESTIMATOR:
    def __init__(self, robot):
        self.robot = robot

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.robot)
        self.tfx = [0.0,0.0,0.0] 
        self.jump = False
        self.odom = np.array([0.0, 0.0, 0.0])
        hz = 10
        self.robot.create_timer(1 / hz, self.set_state)  # TODO

    def set_state(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform("map", "base_footprint", now)
            # print(f"Transform: {trans.transform}")
            self.tfx[0] = trans.transform.translation.x
            self.tfx[1] = trans.transform.translation.y
            self.tfx[2] = 2 * np.arctan2(
                trans.transform.rotation.z, trans.transform.rotation.w
            )  # th =  2*atan(sin(th/2)/cos(th/2))
            d = np.linalg.norm(self.robot.x[0:2] - self.tfx[0:2],ord=2)
            if (d > 0.2) or (d == 0): 
                self.jump = True
            else:
                self.jump = False    

            if self.jump:
                self.robot.controller.maxv = 0.1  # 1.4 # m/s : average walking speed
                self.robot.controller.maxw = 0.2  # np.pi/2 #self.maxv/(0.272/2) # megarover wheel base = 0.272
                self.robot.x[0:3] = self.x[0:3] + (self.robot.rover.odom[0:3] - self.odom[0:3])
            else:
                self.robot.controller.maxv = 0.8  # 1.4 # m/s : average walking speed
                self.robot.controller.maxw = 0.5  # np.pi/2 #self.maxv/(0.272/2) # megarover wheel base = 0.272
                self.odom[0:3] = self.robot.rover.odom[0:3]            
                self.robot.x[0:3] = self.tfx[0:3]
        except Exception as e:
            self.robot.get_logger().warn(f"Could not transform: {e}")

    # def set_pose_callback(self, msg):
    #     print(f"=========")#estimator:{msg.pose}")
    # https://github.com/SteveMacenski/slam_toolbox
    # self.robot.x[0] = msg.pose.pose.position.x
    # self.robot.x[1] = msg.pose.pose.position.y
    # self.robot.x[2] = 2 * np.arctan2(
    #     msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
    # )  # th =  2*atan(sin(th/2)/cos(th/2))
    # msg.header.stamp.nsecs : Int time stamp
    # msg.pose.covariance # : float64[36]
    # print(f"=========estimator:{self.robot.x}")

    def set_pose_callback_matlab(self, msg):  # TO MATLAB
        #     print("dummy")
        # print(f"=========estimator:{msg}")

        self.robot.x[0] = msg.x
        self.robot.x[1] = msg.y
        self.robot.x[2] = msg.z
