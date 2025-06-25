import numpy as np
import math
from rf.rf_robot.robot.gen_sequence import gen_sequence
from rf.acsl_modules.json_load import json_load


class REFERENCE:
    def __init__(self, robot):
        self.robot = robot
        self.ref = self.robot.ref
        self.x = self.robot.x
        th = 0.0  # 67 # SLAM結果のマップに合わせて調整する
        self.offset = np.array([0.0, 0.0])
        self.R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        self.route = []
        # floor 移動用
        self.Bld = json_load("floor_map")  # get floor geo info
        self.brake_rate = 0.0

    def gen_ref(self):
        # print("GEN_REF")
        # generate current referene point
        # cid : current nearest id
        # nid : next id (reference point)
        # gid : goal id in the floor
        print(
            f"Generate reference: [cid,gid,cfloor,tfloor,tid] = [{self.robot.cid},{self.robot.gid},{self.robot.floor},{self.robot.target_floor},{self.robot.target_id}]"
        )
        if self.robot.cid == self.robot.gid:
            # for path through to elevator
            # print(f"0 nid:{self.robot.nid}, cid:{self.robot.cid}, ref:{self.ref}")
            self.robot.nid = self.robot.cid
            # print(f"1 nid:{self.robot.nid}, cid:{self.robot.cid}, ref:{self.ref}")
            self.ref[0:2] = self.P[self.robot.nid - 1, :]  # ref direction is the same
            print(f"p:{self.x}, ref:{self.ref}")
        else:
            self.route = gen_sequence(self.robot.cid, self.robot.gid, self.M)
            print(f"Generated route: {self.route}")
            self.ref[0:2] = self.P[self.robot.cid - 1, :]  # [x,y]
            # print(f"2 nid:{self.robot.nid}, cid:{self.robot.cid}, ref:{self.ref}")
            self.robot.nid = self.route[1]
            # print(f"3 nid:{self.robot.nid}, cid:{self.robot.cid}, ref:{self.ref}")
            # if len(self.route) > 2:
            #     self.ndir =

            if not self.robot.check_position():
                # print(f"4 nid:{self.robot.nid}, cid:{self.robot.cid}, ref:{self.ref}")
                nnid = self.robot.nid
                self.robot.nid = self.robot.cid
                print(
                    f"Set reference to NEAREST point: nid:{self.robot.nid}, cid:{self.robot.cid}, ref:{self.ref}"
                )
            else:
                # print(f"5 nid:{self.robot.nid}, cid:{self.robot.cid}, ref:{self.ref}")
                self.ref[0:2] = self.P[self.robot.nid - 1, :]
                if len(self.route) > 2:
                    nnid = self.route[2]
                else:
                    nnid = self.robot.nid
                print(
                    f"Set reference to NEXT point: nid:{self.robot.nid}, cid:{self.robot.cid}, ref:{self.ref}"
                )

            ### set brake_rate ######
            # tmp: cid to nid,  tmp2: nid to nnid
            # 急カーブ or gidの場合(nnid = nid)に減速する
            self.brake_rate = 0.0
            tmp = self.P[self.robot.nid - 1, :] - self.P[self.robot.cid - 1, :]
            tmpn = np.linalg.norm(tmp, ord=2)
            if tmpn > 0.0:
                tmp /= tmpn        
            # self.ref[2] = np.arctan2(tmp[1],tmp[0])
                tmp2 = self.P[nnid - 1, :] - self.P[self.robot.nid - 1, :]
                tmp2n = np.linalg.norm(tmp2, ord=2)
                if tmp2n > 0.0:
                    tmp2 /= tmp2n
                # print(f"tmp: {tmp} tmp2: {tmp2}")
                    self.brake_rate = tmp @ tmp2

            # self.brake_rate =(tmp[0]*tmp2[0] + tmp[1]*tmp2[1])

        # TO MATLAB
        # message = Point()
        # message.x = self.ref[0]
        # message.y = self.ref[1]
        # message.z = self.ref[2]
        # self.pub.ref.publish(message)

    def update_floor(self, floor, way):
        # After move to a floor, update floor geo info
        # floor : new floor
        # way : way to the floor
        print(
            f"UPDATE FLOOR: from {self.robot.floor} to {floor} by {way}"
        )
        M = np.array(self.Bld[str(floor)]["adjacency"], dtype=np.float64)
        P = np.array(self.Bld[str(floor)]["node"], dtype=np.float64)
        # 距離込みのM行列生成
        for i in range(M.shape[0]):
            i1 = np.zeros((M.shape[0], 1))
            i1[i] = 1
            Mi = np.where(M.dot(i1))[0]
            M[Mi, i] = np.linalg.norm(P[Mi, :] - P[i, :], axis=1)
        self.M = M
        self.P = (self.R @ ((P + self.offset).T)).T  # マップに合わせて目標点を回転
        if way == "elevator":
            # Arrived at the floor by elevator
            self.robot.cid = self.Bld[str(floor)]["elevator"][0]
            # TODO: multiple elevators
            self.robot.nid = self.Bld[str(floor)]["elevator_hall"][0]
        elif way == "init" or way == "go home":
            self.robot.nid = self.robot.cid
        else:  # TODO Not considered yet
            self.robot.cid = self.Bld[str(floor)][way][0]
            self.robot.nid = self.robot.cid
        self.robot.floor = floor
        print(
            f"cid:{self.robot.cid}, nid:{self.robot.nid}, gid:{self.robot.gid}, floor:{self.robot.floor}"
        )
        self.x[0:2] = self.P[self.robot.cid - 1, :]
        tmp = self.P[self.robot.nid - 1, :] - self.P[self.robot.cid - 1, :]
        #self.x[2] = np.arctan2(tmp[1], tmp[0])
        self.set_gid()
        self.gen_ref()

    def avoid_stack(self):
        rads, ds = self.robot.rplidar.find_open_space()
        if not math.isnan(rads):
            em = np.array([np.cos(rads), np.sin(rads)])  # 真ん中方向の単位ベクトル
            ref = self.ref[0:2] - self.robot.x[0:2]
            er = ref / np.linalg.norm(ref, ord=2)  # 目標方向の単位ベクトル
            # er = np.array([-0.5,0.5])
            ip = er @ em  # 目標方向との内積
            ids5 = np.where(ip > 0.1)[0]  # 前方から探す TODO
            im = np.argmax(ip[ids5])  # 目標方向に一番近い範囲インデックス
            # print(f"{em},{ip},{np.argmax(ip[ids5])},{ds[im]},{[np.cos(rads[im]),np.sin(rads[im]),0]}")
            th = self.x[2]
            R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])

            self.ref = np.array(
                ds[im] * (R @ np.array([np.cos(rads[im]), np.sin(rads[im]), 0.0]))
            )  # 新しい目標位置

    def check_gid(self, gid):
        return self.robot.gid == gid
    def check_cid(self, cid):
        return self.robot.cid == cid
    def set_gid(self, gid=0):
        # print("SET_GID")
        # Set goal id in the floor
        # Supposed transition: Dock => elevator_hall => elevator => target_id
        # On target floor, don't need to set elevator hall as gid because of no elevator call
        if gid == 0:
            if self.robot.target_floor == self.robot.floor:
                self.robot.gid = self.robot.target_id
            elif self.check_cid(self.Bld[str(self.robot.floor)]["elevator_hall"][0]):
                self.robot.gid = self.Bld[str(self.robot.floor)]["elevator"][0]
            else:
                self.robot.gid = self.Bld[str(self.robot.floor)]["elevator_hall"][0]
        else:
            self.robot.gid = gid