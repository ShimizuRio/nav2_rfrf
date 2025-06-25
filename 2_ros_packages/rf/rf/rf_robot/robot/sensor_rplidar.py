import rclpy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
import tf_transformations as transformations
from time import time
import numpy as np
from numpy import matlib
import copy


class SENSOR_RPLIDAR:
    def __init__(self, robot):
        self.robot = robot

        # RPLiDAR : [-pi, pi]を1080に反時計回りに分割
        #           pi がインデックス1080 で前
        self.scan = None
        self.fscan = None
        self.bscan = None
        # frontから見てボディで隠れる領域：レーザーindexの範囲
        # 540 が真後ろ 90個で30度
        # self.amin = 405  # 真後ろから-45度
        # self.amax = 685  # 真後ろから+45度
        self.amin = 270  # 真後ろから-45度
        self.amax = 720  # 真後ろから+45度
        dth = 0.005806980188935995  # angle increment
        angle_min = -3.1241390705108643
        L = 0.44  # two rplidar distance [m]
        # alpha: angle in front_rplidar frame ( same dir with robot )
        # [amin, 0]
        al1 = (dth * np.arange(self.amin - 1, 539) + angle_min).reshape(-1, 1)
        # [0, amax]
        al2 = (dth * np.arange(540, self.amax) + angle_min).reshape(-1, 1)

        ra = 360  # 90度分のインデックス
        self.thid1 = 1080 - ra - 1
        self.thid2 = ra
        # theta: angle in back_rplidar frame ( inverse dir with robot )
        # [-pi/2, 0]
        th1 = self.id2rad(dth, angle_min, np.arange(self.thid1, 1080 - 1)) - np.pi
        # [0, pi/2]
        th2 = self.id2rad(dth, angle_min, np.array(range(0, self.thid2))) + np.pi

        phi1 = th1 - al1  # matrix: row: alpha, col: th
        phi2 = th2 - al2
        self.tld = 0.05  # threshold for matching d2 and d2*
        sinphi1 = np.sin(phi1)
        cosphi1 = np.cos(phi1)
        sinal1 = np.sin(al1)
        sinth1 = np.sin(th1)
        cosal1 = np.cos(al1)
        costh1 = np.cos(th1)
        # Confirm element-wise calculation
        self.A1 = L * sinal1 / sinphi1  # d2* : geometric intersection
        self.B1 = L * (sinth1 - sinal1 * cosphi1) / sinphi1
        self.C1 = cosphi1
        # self.B1 = np.hstack([L / cosal1] * len(th1))
        # self.C1 = costh1 / cosal1
        # self.B1 = np.zeros(phi1.shape)
        # self.C1 = sinth1 / sinal1
        self.IDM1 = np.matlib.repmat(
            np.arange(0, self.A1.shape[1]), self.A1.shape[0], 1
        )  # = [[0,1,2,3,...],[0,1,2,3,...],...]

        sinphi2 = np.sin(phi2)
        cosphi2 = np.cos(phi2)
        sinal2 = np.sin(al2)
        sinth2 = np.sin(th2)
        cosal2 = np.cos(al2)
        costh2 = np.cos(th2)
        # Confirm element-wise calculation
        self.A2 = L * sinal2 / sinphi2  # d2
        self.B2 = L * (sinth2 - sinal2 * cosphi2) / sinphi2
        self.C2 = cosphi2
        # self.B2 = np.hstack([L / cosal2] * len(th2))
        # self.C2 = costh2 / cosal2
        # self.B2 = np.zeros(phi2.shape)
        # self.C2 = sinth2 / sinal2
        self.IDM2 = np.matlib.repmat(
            np.arange(0, self.A2.shape[1]), self.A2.shape[0], 1
        )  # = [[0,1,2,3,...],[0,1,2,3,...],...]

        # self.tf_buffer = {}
        self.robot.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.robot.tf_buffer, self.robot)

        self.vp = 0.0
        # print(f" al1: {al1}   al2: {al2}   th1: {th1}   th2: {th2}")
        hz = 10
        self.robot.create_timer(1 / hz, self.merge_scan)  # TODO

    def lidarF_callback(self, msg):
        self.fscan = msg

    def lidarB_callback(self, msg):
        self.bscan = msg

    def merge_scan(self):
        if self.fscan is None or self.bscan is None:
            print("RPLidar is down!!!!!!!!!!!!!!!!")
            return

        merged_scan = copy.deepcopy(self.fscan)
        ranges = np.array([copy.deepcopy(self.fscan.ranges)])
        ranges[0, self.amin - 1 : self.amax] = np.inf  # initialize hidden range

        # al: [-amin, 0]  th: [-pi/2, 0 ] の範囲
        D1 = np.array([self.bscan.ranges[self.thid1 : 1079]])  # distance by bscan
        cond1 = (D1 < self.A1) & (
            self.A1 - D1 < self.tld
        )  # same size with A1: A1 - tld < D1 < A1
        nonzero_id1 = np.any(cond1, axis=1)  # True: satisfy condition
        if np.any(nonzero_id1) > 0:  # True case
            J1 = np.argmin(
                np.where(cond1[nonzero_id1, :], self.IDM1[nonzero_id1, :], 1080), axis=1
            )
            # J1 = np.argmax(cond1[nonzero_id1, :], axis=1)
            ranges[0, self.amin - 1 : 539][nonzero_id1] = (
                D1[0, J1] * self.C1[nonzero_id1, J1] + self.B1[nonzero_id1, J1]
            )

        # al: [0, amax]  th: [ 0, pi/2 ] の範囲
        D2 = np.array([self.bscan.ranges[0 : self.thid2]])
        cond2 = (D2 < self.A2) & (
            self.A2 - D2 < self.tld
        )  # same size with A2 = al x th
        nonzero_id2 = np.any(cond2, axis=1)  # row: boolean,  True: al satisfy cond2
        # print(f"D2: {D2.shape}, inf:{nonzero_id2}, cond:{cond2}")
        if np.any(nonzero_id2) > 0:  # for al that satisfy cond2
            J2 = np.argmax(
                np.where(cond2[nonzero_id2, :], self.IDM2[nonzero_id2, :], -1), axis=1
            )
            # print(f" J2: {J2.shape}, D2[J2]: {D2[0,J2].shape}")
            ranges[0, 540 : self.amax][nonzero_id2] = (
                D2[0, J2] * self.C2[nonzero_id2, J2] + self.B2[nonzero_id2, J2]
            )

        #        ranges[0, ranges[0] == np.inf] = merged_scan.range_max + 20
        # ranges[0, ranges[0] < merged_scan.range_min] = merged_scan.range_max + 20
        merged_scan.ranges = [float(value) for value in ranges[0]]
        merged_scan.header.stamp = self.robot.get_clock().now().to_msg()
        self.merged_scan = merged_scan
        self.robot.pub.scan.publish(merged_scan)
        # print(f"merge: {merged_scan.ranges}")
        # print(f"fscan: {self.fscan.ranges}")

    def id2rad(self, angle_increment, angle_min, id):
        # for scan data
        # pi: front, 0: back
        rad = id * angle_increment + angle_min
        return rad

    def check_front_safty(self, v, safty_range):
        if self.fscan:
            m, rad = self.front_min(safty_range["front_range"] / 2)
            # consider triangle: current position O, arc by v and w with dt (A,B)
            # angle AOB = center angle of the arc = front range
            vp = self.robot.controller.input[0]
            if (
                m - v > safty_range["front"][0] * vp + safty_range["front"][1]
            ):  # move by inertia + mergin
                return 1, 0, 0  # safe
            else:
                risk = safty_range["front"][0] * vp + safty_range["front"][1] + v - m
                # / (
                #     safty_range["front"][0] * vp + safty_range["front"][1]
                # )  # [0,1]
                return (
                    1
                    - risk
                    / (safty_range["front"][0] * vp + safty_range["front"][1] + v),
                    risk,
                    rad,
                )  # risky : decelerate
        else:
            return 0, 0, 0  # for error avoidance

    def front_min(self, rad):
        # d, d_rad = self.front_min(rad)
        # rad: left/right range in radian from front to be care
        # d: minimum distance
        # d_rad: radian of direction d: see id2rad
        # 前方120度の領域
        num = round(3 * rad * 180 / np.pi)  # 目安180 = 60度程度
        a = 2.0 / (num * num)
        w = 1.0 + a * np.arange(0, num) ** 2.0  # num の位置で2倍換算：2乗で緩和
        # print(f"w: {w.shape}, num:{num}")
        fdata = np.concatenate(
            (
                self.fscan.ranges[0:num] * w,
                w[::-1] * self.fscan.ranges[1080 - num : 1080],
            )
        )
        id = np.argmin(fdata)
        return fdata[id], self.id2rad(
            self.fscan.angle_increment, self.fscan.angle_min, id
        )

    def find_open_space(self):
        dth = self.merged_scan.angle_increment
        angle_min = self.merged_scan.angle_min
        org = copy.copy(self.merged_scan.ranges)

        nex = np.concatenate((copy.copy(org[1:]), [org[0]]))
        pids = np.where(nex - org > 1.0)[0]  # 反時計回りで値が飛ぶところ TODO
        pids = pids[np.where(~(org[pids] > 1.0))[0]]  # 値が1m以上飛ぶ点 TODO
        nids = np.where(org - nex > 1.0)[0]  # 時計回りで値が飛ぶところ TODO
        nids = nids[np.where(~(org[nids + 1] > 1.0))[0]]  # 値が1m以上飛ぶ点 TODO
        ids = np.concatenate((pids, nids + 1))
        ids.sort()
        ranges = [
            org[ids[id - 1] + 1 : ids[id]] for id in range(1, len(ids))
        ]  # 値が飛ぶ一つ内側の範囲を抽出
        ranges.append(
            np.concatenate((org[ids[-1] + 1 :], org[0 : ids[0]]))
        )  # 0をまたぐ領域を追加
        ids2 = np.where([len(r) > 10 for r in ranges])[
            0
        ]  # レーザー10本以上の幅の範囲のインデックス TODO
        ranges2 = [ranges[i] for i in ids2]  # 狭すぎる範囲は削除
        ids3 = np.where(~np.array([np.any(r < 1.0) for r in ranges2]))[
            0
        ]  # １m以内の点を含まない範囲のインデックス TODO
        ranges3 = [ranges2[i] for i in ids3]  # ひらけている区間を抽出

        I2 = [ids2[i] for i in ids3]  # ids for ids
        if I2[-1] == len(ids):  # 0をまたぐ領域を考慮
            ranges4 = [org[ids[i] : ids[i + 1] + 1] for i in I2[:-1]]
            ranges4.append(np.concatenate((org[ids[-1] :], org[0 : ids[0] + 1])))
        else:
            ranges4 = [org[ids[i] : ids[i + 1] + 1] for i in I2]
        ids4 = np.where(
            [
                (r[0] ** 2 + r[-1] ** 2 - 2 * r[0] * r[-1] * np.cos(dth * len(r)))
                > 0.025
                for r in ranges4
            ]
        )[
            0
        ]  # ひらけている区間の幅が0.5m以上の領域を抽出 dth*len(r): なす角 余弦定理より TODO
        I3 = np.array([I2[i] for i in ids4])  # ids for ids
        # ids[I3]: 実際のレーザーインデックス
        # print(f"{I3+1}")
        if len(I3) > 0:
            # 開けた範囲の真ん中方向の角度
            rads = angle_min + dth * (ids[I3] + ids[I3 + 1]) / 2 + np.pi
            # 開けた範囲の真ん中までの距離
            ds = (org[ids[I3]] + org[ids[I3 + 1]]) / 2
            return rads, ds
        else:
            return np.nan, np.nan

    def destroy_subscription(self):
        self.robot.sub.destroy_subscription()
