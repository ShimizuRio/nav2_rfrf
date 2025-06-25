#! /bin/python3
import rclpy
from rclpy.node import Node
from bluepy.btle import Scanner, DefaultDelegate
from rclpy.action import ActionServer, ActionClient
from std_msgs.msg import String, Int8, Bool, Int32MultiArray, Float32MultiArray
from rclpy.qos import qos_profile_sensor_data
import os
import time
import numpy as np
import sys

os.chdir(os.path.dirname(os.path.abspath(__file__)))  # カレントディレクトリの移動
sys.path.append(os.path.join(os.path.dirname(__file__)))  # パスの追加

from rclpy.callback_groups import ReentrantCallbackGroup
from switchbot_interfaces.action import Switchbot

# from rf_interfaces.srv import RequestBuilding
from rf_interfaces.action import RequestBuilding
from rf.acsl_modules.json_load import json_load

## Error raised in execute callback: 'RequestBuilding_Goal' object has no attribute 'publish_feedback'

class MAIN(Node):
    def __init__(self):
        super().__init__("building_node")
        self.Bld = json_load("switchbot")
        self.Bots = self.Bld["Bots"]
        self.target_floor = np.nan
        self.floor = np.nan

        # アクションサーバーの初期化 : for robot
        self._building_action_server = ActionServer(
            self,
            RequestBuilding,
            "/building_node/RequestBuilding",
            self.building_request_handler,
            callback_group=ReentrantCallbackGroup(),  # parallel exec
        )
        # サービスサーバの初期化 : for robot

        # to switchbot
        # アクションクライアントの初期化
        self._bot_action_client = ActionClient(self, Switchbot, "/BLD/Switchbot")

        # VL
        self._vl_reciever = self.create_subscription(
            Float32MultiArray,
            "~/vl53l1x_node/range",
            self.vl_callback,
            qos_profile_sensor_data,
        )

    def building_request_handler(self, goal_handle):
        self.get_logger().info(f"Recieve request: {goal_handle}")
        # To extend the function set here
        if (
            goal_handle.request.request == "call elevator"
        ):  # data = [current floor, target floor]
            feedback_msg = self.call_elevator(goal_handle)
        if (
            goal_handle.request.request == "move elevator"
        ):  # data = [current floor, target floor]
            feedback_msg = self.move_elevator(goal_handle)
        result = RequestBuilding.Result()  # 結果メッセージの作成
        result.result = feedback_msg.progress
        return result

    def call_elevator(self, goal_handle):
        # data = [current floor, target floor]
        data = goal_handle.request.data
        self.get_logger().info(f"Call elevator to {data}")

        # action
        if data[0] < data[1]:  # エレベータを呼ぶ：上の階へ行くとき
            s = "Up" + str(data[0])
        else:
            s = "Down" + str(data[0])

        self.get_logger().info("Calling elevator : ")
        self.result = self.drive_bot(s)

        # feedback
        feedback_msg = RequestBuilding.Feedback()
        feedback_msg.progress = "Calling ..."
        goal_handle.publish_feedback(feedback_msg)

        # result
        if self.result:
            goal_handle.succeed()
            feedback_msg.progress = "Success!"
        else:
            goal_handle.abort()
            feedback_msg.progress = "Failure..."

        return feedback_msg

    def move_elevator(self, goal_handle):
        # In the elevator
        data = goal_handle.request.data
        self.get_logger().info("Move elevator to {data[1]}")

        # action : push target floor button
        s = "l" + str(data[1]) # TODO: left and right elevator
        self.result = self.drive_bot(s)
        # if self.result:
        #     # waiting to arrive at the target floor
        #     self.result = self.waiting_elevator_open()
        # else:
        #     goal_handle.abort()
        #     feedback_msg.progress = "Failure..."

        # feedback
        feedback_msg = RequestBuilding.Feedback()
        feedback_msg.progress = "Moving ..."
        goal_handle.publish_feedback(feedback_msg)

        # result
        if self.result:
            goal_handle.succeed()
            feedback_msg.progress = "Success! Arrived at the target floor"
        else:
            goal_handle.abort()
            feedback_msg.progress = "Failure..."

        return feedback_msg

    def waiting_elevator_open(self):
        tcout = 0
        while self.vl[0] < 3 or tcout > 300:
            time.sleep(1.0)  # 1.0秒停止
            tcout += 1
        if tcout <= 300:
            return 1
        else:
            return 0

    def vl_callback(self, topic):
        self.vl = topic.data
        self.vl_received = 1

    ### switchbot #######################
    def drive_bot(self, str):
        self.drive_bot_mac_address(self.Bots[str])
        return True

    def drive_bot_mac_address(self, mac_address):
        self.get_logger().info(mac_address)
        # to switchbot
        bot_goal_msg = Switchbot.Goal()
        bot_goal_msg.mac_address = mac_address

        self.get_logger().info("Waiting for action server...")
        self._bot_action_client.wait_for_server()

        self.get_logger().info("Sending goal request...")

        self._bot_goal_handle = self._bot_action_client.send_goal_async(
            bot_goal_msg, feedback_callback=self.bot_feedback_callback
        )
        self._bot_goal_handle.add_done_callback(self.bot_goal_response_callback)

    def bot_goal_response_callback(self, future):
        # callback at receiving goal response
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        self._bot_get_result = goal_handle.get_result_async()
        self._bot_get_result.add_done_callback(self.bot_get_result_callback)

    def bot_get_result_callback(self, future):
        # callback at receiving result
        result = future.result().result
        #print(f"{future}, {future.result()}, {result}")
        self.get_logger().info(result.result)
        # if result.result == "Success!":
        #     self.floor = self.target_floor
        #     # self.floor = 4 #デモ用に4階に固定
        #     self.target_floor = np.nan
        # elif result.result == "switchbot didn't work":
        #     self.target_floor = np.nan
        #     time.sleep(1)
    

    def bot_feedback_callback(self, feedback_msg):
        # callback at receiving feedback
        feedback = feedback_msg.feedback
        self.get_logger().info(feedback.progress)


def main(args=None):
    rclpy.init(args=args)
    # time.sleep(5.0)
    controller = MAIN()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
