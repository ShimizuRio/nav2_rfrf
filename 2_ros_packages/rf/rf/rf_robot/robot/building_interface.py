from rclpy.action import ActionClient
from rf_interfaces.action import RequestBuilding
from time import sleep

class BUILDING_INTERFACE:
    def __init__(self, robot):
        self.robot = robot
        # アクションクライアントの初期化
        self._action_client = ActionClient(
            self.robot, RequestBuilding, "/building_node/RequestBuilding"  # TODO : hostname
        )
        self.action_flag = ""

##########################################
## Action to Building
##########################################

    def send_goal(self,request,data):
        # 
        self.action_flag = "0"
        goal_msg = RequestBuilding.Goal()
        goal_msg.request = request
        goal_msg.data = data
        #goal_msg.order = [self.floor, self.target_floor,self.request]

        self.robot.get_logger().info(f"Access building for {request}: Waiting for action server...")
        self._action_client.wait_for_server()

        self.robot.get_logger().info("Sending goal request...")
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # callback at receiving goal response
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.robot.get_logger().info("Goal rejected")
            return

        self.robot.get_logger().info("Goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # callback at receiving result
        result = future.result().result
        self.robot.get_logger().info(f"BLD result: {result.result}")
        if "Success" in result.result:
            self.action_flag = "1"
            # if self.robot.status == "call elevator":
            #     #self.robot.status = "move"
            #     self.robot.gid = self.robot.Bld[str(self.robot.floor)]['elevator'][0]
            # if self.robot.status == "move elevator": # After floor move
            #     self.robot.floor = self.robot.target_floor
            #     #self.robot.status = "move"
            #     self.robot.gid = self.robot.target_id
        elif result.result == "building didn't work":
            self.robot.target_floor = 0
            sleep(1)

    def feedback_callback(self, feedback_msg):
        # callback at receiving feedback
        feedback = feedback_msg.feedback
        self.robot.get_logger().info(feedback.progress)
