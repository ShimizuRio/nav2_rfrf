import numpy as np
from geometry_msgs.msg import Twist, Point
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt


class CONTROLLER:
    def __init__(self, robot):
        self.robot = robot
        # controller max translate, angular
        self.maxv = 0.8  # 1.4 # m/s : average walking speed
        self.maxw = 0.5  # np.pi/2 #self.maxv/(0.272/2) # megarover wheel base = 0.272

        #### S safty #############################
        self.safty = {
            "front": [0.1, 0.3],  # dt, mergin
            "front_range": 2 * np.pi / 3,  # 前方120度
        }
        #### E safty #############################
        self.input = [0.0, 0.0]

        self.avoid_stack_start_num = 10
        self.stack_num = 0

    def calc_twist(self):
        message = Twist()
        dir = (self.robot.ref[0:2] - self.robot.x[0:2]).astype(float) @ np.array(
            [
                [np.cos(self.robot.x[2]), -np.sin(self.robot.x[2])],
                [np.sin(self.robot.x[2]), np.cos(self.robot.x[2])],
            ]
        )  # >= 0.2:
        norm = np.linalg.norm(dir, ord=2)
        if norm != 0:
            dir /= norm
        else:
            print("The norm is zero, cannot normalize the vector.")

        # 行きたい方向に応じて速度・角速度を変化
        # exp内の-がついている係数が大きいほど急峻な変化（linearはdeadzon長めにangularはゲイン高めの設計になる）
        message.linear.x = min(self.maxv, (1 / (1 + np.exp(-10 * (dir[0] - 0.8)))))
        message.angular.z = max(
            -self.maxw, min(self.maxw, (2 / (1 + np.exp(-4 * (dir[1]))) - 1))
        )  # self.maxw*

        #### for safty ############################

        k, risk, rad = self.robot.rplidar.check_front_safty(
            message.linear.x, self.safty
        )

        # print(
        #     f"0 risk:{risk}, k:{k}, rad:{rad}, x:{message.linear.x}, z:{message.angular.z}"
        # )

        k = max(-0.1, k)

        # if k < 1:
        #    print(f"EMG: too close to front object : {k}")
        # stop for collision risk
        # 直角に曲がる時やgidポイントではポイントまでの距離に応じた減速をする
        if self.robot.reference.brake_rate < 0.5:  #
            # print(f"norm: {norm}")
            message.linear.x *= min(1.0, norm + 0.2)  # 0.2は減速しすぎないための定数
        # message.angular.z *= k
        message.linear.x -= risk
        # message.angular.z *= k  # too conserva?
        # if message.linear.x < 0.4 and np.abs(message.angular.z) < 0.02:
        # self.input = [0.5, message.angular.z] # for debug
        # 危険判定では停止
        if risk > 0.9 or (risk > 0.5 and np.abs(message.angular.z) < 0.01):
            print(
                f"    Too close: {k}, vx: {message.linear.x}, vyaw: {message.angular.z}"
            )
            message.linear.x = 0.0
            message.angular.z = 0.0
            if self.stack_num > self.avoid_stack_start_num:
                self.robot.reference.avoid_stack()  # set new ref
            self.stack_num += 1
        else:
            self.stack_num = 0

        # print(
        #     f"1 risk:{risk}, k:{k}, rad:{rad}, x:{message.linear.x}, z:{message.angular.z}"
        # )
        self.input = [message.linear.x, message.angular.z]  # for debug
        return message
