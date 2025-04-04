import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class Nav2Toggle(Node):
    def __init__(self):
        super().__init__('nav2_toggle')
        self.nav2_active = False
        self.start_time = self.get_clock().now()
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.change_state_cli = self.create_client(ChangeState, '/lifecycle_manager_navigation/change_state')
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.toggle_nav2(False)  # Start with Nav2 off

    def toggle_nav2(self, activate):
        if activate == self.nav2_active:
            return
        req = ChangeState.Request()
        req.transition.id = 3 if activate else 2  # 3 = activate, 2 = deactivate
        self.change_state_cli.call_async(req)
        self.nav2_active = activate

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = -2.0  # Goal at (0,0)
        goal_msg.pose.pose.position.y = -0.0
        goal_msg.pose.pose.orientation.w = 1.0
        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal_msg)

    def timer_callback(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9  # Seconds
        if elapsed < 10.0:
            twist = Twist()
            twist.linear.x = 0.2  # Publish x = 0.2 when Nav2 is off
            self.cmd_vel_pub.publish(twist)
        elif not self.nav2_active:
            self.toggle_nav2(True)  # Turn Nav2 on after 10s
            self.send_goal()  # Navigate to (0,0)

def main():
    rclpy.init()
    node = Nav2Toggle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
