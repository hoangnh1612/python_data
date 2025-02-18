#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import math

class Nav2Controller(Node):
    def __init__(self):
        super().__init__('nav2_controller')
        
        # Callback groups
        self.nav_cb_group = MutuallyExclusiveCallbackGroup()
        self.sensor_cb_group = ReentrantCallbackGroup()
        
        # Lưu trữ target và trạng thái
        self.target_pose = None
        self.is_paused = False
        self.navigation_in_progress = False
        
        # Action client cho navigation
        self.nav_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose',
            callback_group=self.nav_cb_group
        )
        
        # Publisher để dừng robot
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10,
            callback_group=self.sensor_cb_group
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10,
            callback_group=self.sensor_cb_group
        )
        
        # Các thông số
        self.min_safe_distance = 0.5  # meters
        self.front_angle_range = 60   # degrees
        
        self.get_logger().info('Nav2Controller initialized')

    def navigate_to_pose(self, pose_msg):
        """Bắt đầu navigation đến pose mới"""
        self.get_logger().info('Navigating to new pose')
        
        # Tạo và gửi goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg
        
        self.navigation_in_progress = True
        self.is_paused = False
        
        # Gửi goal và lưu future
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback khi nhận được response từ action server"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            self.navigation_in_progress = False
            return

        # Lấy kết quả của goal
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Callback khi goal hoàn thành"""
        self.navigation_in_progress = False
        self.is_paused = False
        self.get_logger().info('Navigation completed')

    def goal_callback(self, msg):
        self.target_pose = msg
        self.navigate_to_pose(msg)

    def pause_navigation(self):
        if not self.is_paused:
            self.get_logger().info('Pausing navigation due to obstacle...')
            
            # Gửi lệnh dừng
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.linear.y = 0.0
            stop_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_cmd)
            
            self.is_paused = True

    def resume_navigation(self):
        if self.is_paused and self.target_pose:
            self.get_logger().info('Resuming navigation to saved target...')
            self.navigate_to_pose(self.target_pose)

    def scan_callback(self, msg):
        try:
            # Tính toán các index cho góc quét phía trước
            angle_range = len(msg.ranges)
            angle_min = msg.angle_min
            angle_max = msg.angle_max
            
            front_angle = math.radians(self.front_angle_range / 2)
            points_per_radian = angle_range / (angle_max - angle_min)
            center_idx = angle_range // 2
            points_per_side = int(front_angle * points_per_radian)
            
            start_idx = max(0, center_idx - points_per_side)
            end_idx = min(angle_range, center_idx + points_per_side)
            
            front_distances = [
                r for r in msg.ranges[start_idx:end_idx]
                if r > 0 and not math.isinf(r) and not math.isnan(r)
            ]
            
            if not front_distances:
                self.get_logger().warn('No valid distance measurements in front sector')
                return
                
            min_front_distance = min(front_distances)
            
            self.get_logger().debug(f'Min front distance: {min_front_distance:.2f}m')
            
            if min_front_distance < self.min_safe_distance and self.navigation_in_progress:
                self.get_logger().warn(f'Obstacle detected at {min_front_distance:.2f}m!')
                self.pause_navigation()
            elif min_front_distance >= self.min_safe_distance and self.is_paused:
                self.get_logger().info('Path clear, resuming navigation.')
                self.resume_navigation()
                
        except Exception as e:
            self.get_logger().error(f'Error in scan_callback: {str(e)}')

def main():
    rclpy.init()
    controller = Nav2Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
