import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class LocalizationSelector(Node):
    def __init__(self):
        super().__init__('localization_selector')
        self.amcl_pose_sub = self.create_subscription(PoseStamped, '/amcl_pose', self.amcl_callback, 10)
        #self.slam_pose_sub = self.create_subscription(PoseStamped, '/slam_pose', self.slam_callback, 10)
        self.publisher = self.create_publisher(PoseStamped, '/selected_pose', 10)
        self.current_pose = None

    def amcl_callback(self, msg):
        # 静的環境ではAMCLのデータを優先
        self.current_pose = msg
        self.publish_pose()

    # def slam_callback(self, msg):
    #     # 動的環境ではSLAM Toolboxのデータを優先
    #     self.current_pose = msg
    #     self.publish_pose()

    def publish_pose(self):
        if self.current_pose:
            self.publisher.publish(self.current_pose)

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationSelector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
