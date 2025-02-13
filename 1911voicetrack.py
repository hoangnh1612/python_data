import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from utils import DictionaryReceiver

def clip(value, min_val, max_val):
    return max(min(value, max_val), min_val)

class VoiceTrackNode(Node):
    def __init__(self):
        super().__init__('voice_track')
        
        # Publisher
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel_3d', 10)
        
        # Voice receiver (port 2012)
        self.receiver_voice = DictionaryReceiver(port=2012)
        self.setup_receiver(self.receiver_voice, "Voice")
        
        # Tracking receiver (port 2011)
        self.receiver_track = DictionaryReceiver(port=2011)
        
        # Tracking parameters
        self.target_distance = 0.75
        self.x_local = self.target_distance
        self.theta_local = 0.0
        
        # PID parameters
        self.prev_x_error = 0.0
        self.prev_theta_error = 0.0
        self.sum_e_x = 0.0
        self.sum_e_theta = 0.0
        self.kp_x = 0.25
        self.kd_x = 0.0
        self.kp_theta = 0.33
        self.kd_theta = 0.0
        self.kix = 0.0
        self.kit = 0.0
        self.dt = 0.1
        
        # Control mode and data storage
        self.tracking_mode = False
        self.previous_voice_command = None
        
        # Timer
        self.timer = self.create_timer(self.dt, self.control_loop)

    def setup_receiver(self, receiver, name):
        try:
            receiver.start_listening()
            receiver.accept_connection()
            self.get_logger().info(f"{name} receiver initialized on port {receiver.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize {name} receiver: {str(e)}")

    def control_loop(self):
        msg = Twist()
        
        if not self.tracking_mode:
            # Handle voice commands
            voice_dict = self.receiver_voice.receive_dictionary()
            if voice_dict is not None:
                self.previous_voice_command = voice_dict
            
            if self.previous_voice_command is not None:
                self.process_voice_command(self.previous_voice_command, msg)
        else:
            track_dict = self.receiver_track.receive_dictionary()
            if track_dict is not None:
                if self.check_stop_tracking(track_dict):
                    self.exit_tracking_mode()
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                else:
                    self.process_track_data(track_dict, msg)
        
        # Publish the velocity command
        self.vel_publisher.publish(msg)

    def check_stop_tracking(self, track_dict):
        return track_dict.get("stop_flag", True)

    def exit_tracking_mode(self):
        self.tracking_mode = False
        self.receiver_track.close()
        self.get_logger().info("Tracking stopped, returning to voice control")
        # Reset all tracking-related variables
        self.x_local = self.target_distance
        self.theta_local = 0.0
        self.prev_x_error = 0.0
        self.prev_theta_error = 0.0
        self.sum_e_x = 0.0
        self.sum_e_theta = 0.0

    def process_voice_command(self, received_dict, msg):
        action = received_dict.get("action", "")
        
        if action == "follow" and not self.tracking_mode:
            self.tracking_mode = True
            self.setup_receiver(self.receiver_track, "Track")  # Open port 2011
            self.previous_voice_command = None  # Clear previous command
            return
        
        # Process other voice commands
        if action == "go":
            msg.linear.x = 0.3
        elif action == "left":
            msg.angular.z = 0.3
        elif action == "right":
            msg.angular.z = -0.3
        elif action == "back":
            msg.linear.x = -0.3
        elif received_dict.get("stop_flag", False):
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.previous_voice_command = None  # Clear previous command on stop

    def process_track_data(self, track_dict, msg):
        try:
            new_depth = track_dict.get("Depth", self.x_local)
            new_angle = -track_dict.get("Angle", self.theta_local)
            
            self.x_local = new_depth
            self.theta_local = new_angle
            
            # Calculate errors
            x_error = self.x_local - self.target_distance
            theta_error = self.theta_local
            
            self.sum_e_x += x_error * self.dt
            self.sum_e_theta += theta_error * self.dt
            
            # PID control for x (distance)
            derivative_x = (x_error - self.prev_x_error) / self.dt
            vel_x = self.kp_x * x_error + self.kd_x * derivative_x + self.kix * self.sum_e_x
            
            # PID control for theta (angle)
            derivative_theta = (theta_error - self.prev_theta_error) / self.dt
            vel_theta = self.kp_theta * theta_error + self.kd_theta * derivative_theta + self.kit * self.sum_e_theta
            
            # Clip velocities and assign to message
            msg.linear.x = clip(vel_x, -0.3, 0.3)
            msg.angular.z = clip(vel_theta, -0.3, 0.3)
            
            # Update previous errors
            self.prev_x_error = x_error
            self.prev_theta_error = theta_error
            
            self.get_logger().debug(f"Distance: {self.x_local:.2f}, Angle: {self.theta_local:.2f}, " 
                                   f"vel_x: {msg.linear.x:.2f}, vel_theta: {msg.angular.z:.2f}")
        except Exception as e:
            self.get_logger().error(f"Error processing tracking data: {str(e)}")

    def close_receivers(self):
        self.receiver_voice.close()
        if self.tracking_mode:
            self.receiver_track.close()
        self.get_logger().info("Closed all receivers")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceTrackNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    finally:
        node.close_receivers()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
