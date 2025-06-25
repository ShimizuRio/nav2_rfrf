#! /bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import sys
import os
import numpy as np
os.chdir(os.path.dirname(os.path.abspath(__file__)))  # カレントディレクトリの移動
sys.path.append(os.path.join(os.path.dirname(__file__)))  # パスの追加
from time import sleep
import subprocess

from rf.acsl_modules.json_load import json_load
from rf.rf_robot.robot.heartbeat import HEARTBEAT
from rf.rf_robot.robot.controller import CONTROLLER
from rf.rf_robot.robot.estimator import ESTIMATOR
from rf.rf_robot.robot.reference import REFERENCE
from rf.rf_robot.robot.sensor_rplidar import SENSOR_RPLIDAR
from rf.rf_robot.robot.rover_interface import ROVER_INTERFACE
from rf.rf_robot.robot.building_interface import BUILDING_INTERFACE
from rf.rf_robot.robot.set_publisher import set_publisher
from rf.rf_robot.robot.set_subscriber import set_subscriber

class ROBOT(Node):

    def __init__(self):
        super().__init__("robot_node")
        print("Init RF ROBOT node")
        
        self.declare_parameter('rid', 1)
        self.declare_parameter('send_input', 0)
        rid = str(self.get_parameter('rid').get_parameter_value().integer_value)
        send_input = self.get_parameter('send_input').get_parameter_value().integer_value
        self.x = np.array([0,0,0], dtype=np.float64)
        self.ref = np.array([0,0,0], dtype=np.float64)
        
        self.frame = np.array([[0.3,0.3],[-0.6,0.3],[-0.6,-0.3],[0.3,-0.3]])
        self.rid = rid # robot id
        self.flag= {"send_input":send_input}
        self.pub = set_publisher(self)
        self.rover = ROVER_INTERFACE(self)
        self.bld = BUILDING_INTERFACE(self)
        self.hb = HEARTBEAT(self)         # 疎通確認用のパラメータ
        self.create_timer(0.1,self.hb.do)
        self.controller = CONTROLLER(self)
        self.estimator = ESTIMATOR(self)
        self.reference = REFERENCE(self)
        self.rplidar = SENSOR_RPLIDAR(self)
        self.sub = set_subscriber(self)
        print(f"---------{rid}, flag:{send_input}")
        

        self.target_floor = np.nan # 
        self.target_id = np.nan # target place id on target floor
        self.reference.set_gid(np.nan) # goal place id on current floor
        self.base=json_load("robot_placement")
        self.floor = self.base[str(self.rid)]['floor'] # current floor
        self.cid = self.base[str(self.rid)]['placeId'] # current place id
        self.nid = self.cid # next place id
        self.reference.update_floor(self.floor,"init") # set self.M
        self.x[2] = self.base[str(self.rid)]['angle'] # current direction
        self.ndir = np.array([0.0,0.0])
        self.status = "standby"
        self.standby()
                        
        hz = 10
        self.create_timer(1.0/hz,self.mainloop)
        
#########################################
## Assets
#########################################        

    def standby(self):
        # stop
        try:
            self.rover.send_input([0.0,0.0])
            self.reference.set_gid(np.nan)
        except Exception as e: 
            self.get_logger().error(f"Error while publishing message: {e}") 

    def set_target(self,data):
        # Set target floor and target id
        # Only called by console
        #print(f"SET TARGET")
        self.target_floor = data[0]
        self.target_id = data[1]
        if len(data) > 2:
            self.floor = data[2]
            self.cid = data[3]
            self.nid = self.cid
            self.x[0:2] = self.reference.P[self.cid-1,:]
            self.ref[0:3] = self.x[0:3] 
            result = subprocess.run(['hostbash', 'drm', 'slam_toolbox'], capture_output=True, text=True)
            print(f"drm: {result}")
            result = subprocess.run(['hostbash', 'dup', 'slam_toolbox', 'localization', "bld10_" + str(self.floor) + "F", str(self.x.tolist())], capture_output=True, text=True)
            print(f"dup: bld10_{str(self.floor)}F {result}")
            self.reference.update_floor(self.floor,"init") # set self.M
        self.reference.set_gid()
        self.reference.gen_ref()
                
    def check_position(self):
        #print("CHECK_POSITION")
        # 0: on the way, 1: reach to target
        th = self.x[2]
        R = np.array([[np.cos(th),-np.sin(th)],[np.sin(th),np.cos(th)]])
        tmpd = np.abs(R.T@(self.ref[0:2].T - self.x[0:2].T)) # body 座標系から見たrefの位置
        #print(f"x:{self.x}, r: {self.ref}, lx : {tmpd}")
        if np.linalg.norm(tmpd,ord=2)<0.4 and tmpd[0] < 0.2:
#        if np.linalg.norm(self.x[0:2] - self.ref[0:2],ord=2) < 0.2:# and dangle < 0.8:
            #print(f"Reach to ref: p:{self.x}, ref:{self.ref}")
            #self.cid = self.nid
            return 1
        else:
            return 0
        
    def move(self):
        message = self.controller.calc_twist()
        try:
            # # for debug: simulation
            # self.x += np.array([message.linear.x*np.cos(self.x[2]),message.linear.x*np.sin(self.x[2]),message.angular.z], dtype=np.float64)*0.1
            # self.x[2] = np.arctan2(np.sin(self.x[2]),np.cos(self.x[2])) # in [-pi, pi]
            #print(f"    u = [{message.linear.x},{message.angular.z}], cp: {self.x}")
            self.rover.send_input([message.linear.x,message.angular.z])
        except Exception as e: 
            self.get_logger().error(f"Error while publishing message: {e}") 

    def handle_error(self): 
        try: # メッセージのデシリアライズ処理 
            pass 
        except Exception as e: 
            self.get_logger().error(f"Error: {e}") 

##########################################
## Listen from Console
##########################################
    def console_listener_callback(self, msg):
        # TODO : error handling
        self.get_logger().info(f"I heard: {msg.data}")
        if msg.data[0] == 0:
            self.rover.send_input([0,0])
            self.reference.set_gid(np.nan)
        else:
            self.set_target(msg.data)

##########################################
## Main Algorithm
##########################################

    def mainloop(self):
        # status transition
        if np.isnan(self.gid):
            self.status = "standby"
        else: # received request
            #print("start")
            if not self.check_position():
                #print("move") 
                # on the way to target
                self.status = "move"
            else: # arrived local target
                self.cid = self.nid
                #print(f"3  gid:{self.gid}, nid:{self.nid}")
                if self.gid != self.nid:
                    # set next local target in current floor
                    print(f"set next local target: current {self.cid}, {self.reference.route}")
                    self.status = "set next target"
                else: # arrived floor target
                    #print("2")
                    if self.floor == self.target_floor and self.reference.check_gid(self.target_id):
                        # arrived global target
                        self.get_logger().info(f"======================================")
                        self.get_logger().info(f"==  Request Complete !!   ============")
                        self.get_logger().info(f"======================================")
                        self.status = "standby"
                    elif self.reference.check_gid(self.reference.Bld[str(self.floor)]['elevator_hall'][0]):
                        if self.floor != self.target_floor: # change floor
                            self.status = "call elevator"
                        else:
                            self.status = "move global target"
                    else: # in elevator
                        self.status = "move elevator"
        
        # Action
        match self.status:
            case "call elevator":
                # Just call an elevator and then start moving to the elevator without checking the elevator isbld_send here. TODO: check the elevator arrival 
                print(f"call ele: {self.bld.action_flag}")
                self.rover.send_input([0,0])
                if self.bld.action_flag == "": 
                    print(f"call ele:1")
                    self.bld.send_goal(self.status,[self.floor,self.target_floor])
                elif self.bld.action_flag == "1": # Success case
                    print(f"call ele:2")
                    self.reference.set_gid()
                    self.reference.gen_ref()
                    self.bld.action_flag = ""
            case "move elevator":
                print(f"MOVE ELE: {self.bld.action_flag}")
                self.rover.send_input([0,0])
                if self.bld.action_flag == "":
                    print(f"move ele: empty")
                    self.bld.send_goal(self.status,[self.floor,self.target_floor])
                elif self.bld.action_flag == "1": # Success case
                    print(f"move ele: 1")

                    self.reference.update_floor(self.target_floor,"elevator") # set x, cid, floor, etc 
                    result = subprocess.run(['hostbash', 'drm', 'slam_toolbox'], capture_output=True, text=True)
                    print(f"drm: {result}")
                    result = subprocess.run(['hostbash', 'dup', 'slam_toolbox', 'localization', "bld10_" + str(self.target_floor) + "F", str(self.x.tolist())], capture_output=True, text=True)
                    print(f"dup: bld10_{str(self.target_floor)}F {result}")
                    sleep(2)

                    #self.ref[2] = 0 # TODO multi elevators dirs
                    self.bld.action_flag = ""
            case "set next target":
                self.reference.gen_ref()
            case "move":
                self.move()
            case "move global target":
                self.reference.set_gid(self.target_id)
                self.reference.gen_ref()
            case "go home":
                self.set_target([self.base[str(self.rid)]['floor'],self.base[str(self.rid)]['placeId']])
            case _: # standby
                self.standby()
                # self.controller.calc_twist() # for debug

    def destroy(self):
        self.sub.destroy_subscription()
        super().destroy()
        
def main(args=None):
    rclpy.init(args=args)

    robot = ROBOT()
    try: 
        rclpy.spin(robot) 
    except KeyboardInterrupt: 
        pass 
    except Exception as e: 
        robot.get_logger().error(f"Unexpected error: {e}") 
    finally: 
        robot.destroy_node() 
        rclpy.shutdown()

if __name__ == "__main__":
    main()