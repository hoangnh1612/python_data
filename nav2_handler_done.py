import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Float32MultiArray

class Nav2Toggle(Node):
    def __init__(self):
        super().__init__('nav2_toggle')
        self.nav2_active = False
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.change_state_cli = self.create_client(ChangeState, '/lifecycle_manager_navigation/change_state')
        self.state_sub = self.create_subscription(Float32MultiArray, 'state', self.state_callback, 10)  # Corrected topic syntax
        self.state = None  # Initialize state
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.toggle_nav2(False)  # Start with Nav2 off

    def toggle_nav2(self, activate):
        if activate == self.nav2_active:
            return
        req = ChangeState.Request()
        req.transition.id = 3 if activate else 2  # 3 = activate, 2 = deactivate
        self.change_state_cli.call_async(req)
        self.nav2_active = activate

    def state_callback(self, msg):
        self.state = msg.data[0] if msg.data else None  # Assume first element controls mode (e.g., 1.0 = Nav2, 0.0 = custom)

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()  # Add timestamp
        goal_msg.pose.pose.position.x = -2.0  # Goal at (-2,0)
        goal_msg.pose.pose.position.y = 0.0 
        goal_msg.pose.pose.orientation.w = 1.0
        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal_msg)
        self.get_logger().info("Sending goal")

    def timer_callback(self):
        if self.state is None: 
            self.get_logger().info("Not activated")
            return
        if self.state == 1.0:  # Nav2 mode
            if not self.nav2_active:
                self.toggle_nav2(True)
            self.send_goal()
        else: 
            if self.nav2_active:
                self.toggle_nav2(False)
            twist = Twist()
            twist.angular.z = 0.2  # Spin at 0.2 rad/s
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Rotating")

def main():
    rclpy.init()
    node = Nav2Toggle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
