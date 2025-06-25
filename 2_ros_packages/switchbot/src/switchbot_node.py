#! /bin/python3
import rclpy
from rclpy.node import Node

# from bluepy.btle import Scanner, DefaultDelegate
from rclpy.action import ActionServer, ActionClient
from std_msgs.msg import String, Int8, Bool, Int32MultiArray
import sys
import os
import time

os.chdir(os.path.dirname(os.path.abspath(__file__)))  # カレントディレクトリの移動
sys.path.append(os.path.join(os.path.dirname(__file__)))  # パスの追加
# parent_dir = os.path.abspath(os.path.join(os.getcwd(), '..'))
# sys.path.append(parent_dir)

from switchbot_controller import trigger_device

from rclpy.callback_groups import ReentrantCallbackGroup
from switchbot_interfaces.action import Switchbot

# 最終的に開発していくバージョン


class MAIN(Node):

    def __init__(self):
        super().__init__("switchbot_node")
        self.result = False

        # アクションサーバーの初期化
        self._action_server = ActionServer(
            self,
            Switchbot,
            "/BLD/Switchbot",
            self.switchbot_callback,
        )
        print("Switchbot node ready!")

    # def switchbot_callback(self):# ゴールが送信されたときに呼ばれるコールバック
    def switchbot_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")
        mac_address = goal_handle.request.mac_address
        self.get_logger().info(mac_address)

        feedback_msg = Switchbot.Feedback()
        feedback_msg.progress = "swichbot startup"
        goal_handle.publish_feedback(feedback_msg)
        print("swichbot startup")

        result = Switchbot.Result()  # 結果メッセージの作成

        feedback_msg.progress = mac_address
        goal_handle.publish_feedback(feedback_msg)
        self.result = trigger_device([mac_address, "Bot", "Press"])

        if self.result:
            goal_handle.succeed()
            result.result = "Success!"
        else:
            goal_handle.abort()
            result.result = "Failure..."
        print(f"{result}")
        return result


def main(args=None):
    rclpy.init(args=args)
    controller = MAIN()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
