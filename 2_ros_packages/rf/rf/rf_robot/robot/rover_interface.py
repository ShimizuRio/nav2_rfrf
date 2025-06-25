from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist
from time import time
import numpy as np
import math


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q


class ROVER_INTERFACE:
    def __init__(self, robot):
        self.robot = robot
        self.input = [0, 0]

        yaw0 = 1.5708
        qz = np.sin(yaw0 / 2)
        qw = np.cos(yaw0 / 2)
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self.robot)
        t = TransformStamped()
        t.header.frame_id = "odom"
        t.header.stamp = self.robot.get_clock().now().to_msg()
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        self.tf_broadcaster.sendTransform(t)

        self.tf0_broadcaster = TransformBroadcaster(self.robot)
        self.tf0 = TransformStamped()
        self.tf0.header.frame_id = "map"
        self.tf0.header.stamp = self.robot.get_clock().now().to_msg()
        self.tf0.child_frame_id = "odom0"
        self.tf0.transform.translation.x = 0.0
        self.tf0.transform.translation.y = 0.0
        self.tf0.transform.translation.z = 0.0
        self.tf0.transform.rotation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        self.tf0_broadcaster.sendTransform(self.tf0)

        self.pre_time = self.robot.get_clock().now().to_msg()
        self.odom = np.array([0.0, 0.0, yaw0])
        self.odom0 = np.array([0.0, 0.0, yaw0])

        hz = 10
        self.robot.create_timer(1 / hz, self.odom_publisher)  # TODO

    def odometry_callback(self, msg):
        # callback と publisherに分けなくてもいいかも。
        ct = self.robot.get_clock().now().to_msg()
        dt = ct.sec - self.pre_time.sec + 1e-9 * (ct.nanosec - self.pre_time.nanosec)
        self.odom[0] = self.odom[0] + dt * msg.linear.x * np.cos(self.odom[2])
        self.odom[1] = self.odom[1] + dt * msg.linear.x * np.sin(self.odom[2])
        self.odom[2] = self.odom[2] + dt * msg.angular.z

        # for debug
        self.odom0[0] = self.odom0[0] + dt * msg.linear.x * np.cos(self.odom0[2])
        self.odom0[1] = self.odom0[1] + dt * msg.linear.x * np.sin(self.odom0[2])
        self.odom0[2] = self.odom0[2] + dt * msg.angular.z
        self.pre_time = ct

    def odom_publisher(self):
        # subscribe msg=/rover_odo
        # https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html
        t = TransformStamped()
        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.robot.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"  #'Robot'+self.robot.rid

        t.transform.translation.x = self.odom[0]
        t.transform.translation.y = self.odom[1]
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, self.odom[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
        # self.odom = np.array([0.0, 0.0, 0.0])

        # for debug
        t0 = TransformStamped()
        # Read message content and assign it to
        # corresponding tf variables
        t0.header.stamp = self.robot.get_clock().now().to_msg()
        t0.header.frame_id = "map"
        t0.child_frame_id = "odom0"

        t0.transform.translation.x = self.odom0[0]
        t0.transform.translation.y = self.odom0[1]
        t0.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, self.odom0[2])
        t0.transform.rotation.x = q[0]
        t0.transform.rotation.y = q[1]
        t0.transform.rotation.z = q[2]
        t0.transform.rotation.w = q[3]
        # Send the transformation
        self.tf0_broadcaster.sendTransform(t0)
        # print(f"ODOM0: {self.odom0}")

    def bumper_callback(self, msg):
        self.robot.bumper = msg

    def send_input(self, lst):
        self.input = lst
        message = Twist()
        message.linear.x = float(lst[0])
        message.angular.z = float(lst[1])
        #print(f"{ self.robot.flag}, u: {lst}")
        if self.robot.flag["send_input"]:
            self.robot.pub.input.publish(message)
