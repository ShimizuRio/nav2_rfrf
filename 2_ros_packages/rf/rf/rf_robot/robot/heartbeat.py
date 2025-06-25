from std_msgs.msg import String
from time import time


class HEARTBEAT:
    def __init__(self, robot):
        self.robot = robot
        self.oldt = time()

    def do(self):
        nowt = time()
        interval = nowt - self.oldt
        self.oldt = nowt
        topic = String()
        topic.data = f"HB {self.robot.rid}: status:{self.robot.status} Goal:[{self.robot.target_floor},{self.robot.target_id}]  Local:[{self.robot.nid},{self.robot.ref}]  Current:[{self.robot.floor},{self.robot.cid},{self.robot.x}]  Route:{self.robot.reference.route}  cU: {self.robot.controller.input}  U: {self.robot.rover.input}"
        print(topic.data)
        self.robot.pub.hb.publish(topic)
        if interval > 2:
            self.robot.get_logger().warn(
                f"Reconnect : interval {interval},  time:{nowt}"
            )
