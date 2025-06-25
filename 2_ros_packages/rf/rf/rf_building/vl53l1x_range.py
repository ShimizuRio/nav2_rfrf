#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
# range.py\n
"""
import rclpy
from rclpy.node import Node
from time import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float32MultiArray
from rclpy.qos import qos_profile_sensor_data

class vlrange(Node):
    def __init__(self):
        super().__init__("vl_node")
        self.create_subscription(
            Float32MultiArray,
            "~/range",
            self.callback,
            qos_profile_sensor_data
        )
        self.received = 0
        self.topic.range = range(5)

        self.ready_f = False
        if not self.ready_f:
            self.check_ready_timar = self.create_timer(
                2.0, self.check_ready_callback)

    def check_ready_callback(self):
        if self.received:
            self.get_logger().info(f"Received topic.")
            self.check_ready_timar.cancel()
            self.ready_f = True
        else:
            self.get_logger().warn('Waiting to receive "%s" topic.' % (self.topicName))

    def is_ready(self):
        return self.ready_f

    def callback(self, topic):
        self.topic.range = topic.data
        self.received = 1

    def do(self):
        self.result = self.topic.range
