#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class SwerveController(Node):
    def __init__(self):
        super().__init__('swerve_controller')
        
        # Subscribe to user input (keyboard/joystick)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # Publishers to Gazebo Controllers
        # Note: These topic names must match your controllers.yaml
        self.pivot_pub = self.create_publisher(Float64MultiArray, '/pivot_controller/commands', 10)
        self.drive_pub = self.create_publisher(Float64MultiArray, '/drive_controller/commands', 10)
        
        # Robot Dimensions (from your URDF)
        self.L = 0.6  # Wheelbase Length
        self.W = 0.4  # Track Width

        self.get_logger().info("Swerve Controller Started")

    def cmd_callback(self, msg):
        # ROS 2 Coordinate System: X is Forward, Y is Left, Z is Up
        vx = msg.linear.x
        vy = msg.linear.y 
        w = msg.angular.z 

        # If robot is stopped, do nothing to prevent wheels snapping to 0
        if vx == 0 and vy == 0 and w == 0:
            self.stop_robot()
            return

        # --- SWERVE KINEMATICS ---
        # Calculate the vector components for each wheel
        # A = vx - w * (L/2)     B = vx + w * (L/2)
        # C = vy - w * (W/2)     D = vy + w * (W/2)
        
        # Note: Dimensions are halved for calculation from center
        half_L = self.L / 2.0
        half_W = self.W / 2.0
        
        # Wheel Velocity Vectors (vx - wy*ry, vy + wx*rx)
        # Front Left (x: +, y: +)
        v_fl_x = vx - w * half_W
        v_fl_y = vy + w * half_L
        
        # Front Right (x: +, y: -)
        v_fr_x = vx - w * (-half_W)
        v_fr_y = vy + w * half_L

        # Back Left (x: -, y: +)
        v_bl_x = vx - w * half_W
        v_bl_y = vy + w * (-half_L)

        # Back Right (x: -, y: -)
        v_br_x = vx - w * (-half_W)
        v_br_y = vy + w * (-half_L)

        # Calculate Polar Coordinates (Speed & Angle)
        # 1. Speeds
        speed_fl = math.sqrt(v_fl_x**2 + v_fl_y**2)
        speed_fr = math.sqrt(v_fr_x**2 + v_fr_y**2)
        speed_bl = math.sqrt(v_bl_x**2 + v_bl_y**2)
        speed_br = math.sqrt(v_br_x**2 + v_br_y**2)

        # 2. Angles (atan2 returns -pi to pi)
        angle_fl = math.atan2(v_fl_y, v_fl_x)
        angle_fr = math.atan2(v_fr_y, v_fr_x)
        angle_bl = math.atan2(v_bl_y, v_bl_x)
        angle_br = math.atan2(v_br_y, v_br_x)

        # Publish Commands
        # ORDER MATTERS: Must match controllers.yaml 
        # (FR, FL, BR, BL)
        
        pivot_msg = Float64MultiArray()
        pivot_msg.data = [angle_fr, angle_fl, angle_br, angle_bl]
        self.pivot_pub.publish(pivot_msg)

        drive_msg = Float64MultiArray()
        drive_msg.data = [speed_fr, speed_fl, speed_br, speed_bl]
        self.drive_pub.publish(drive_msg)

    def stop_robot(self):
        d_msg = Float64MultiArray()
        d_msg.data = [0.0, 0.0, 0.0, 0.0]
        self.drive_pub.publish(d_msg)
        # We don't publish pivot angles when stopping to keep wheels in last known position

def main(args=None):
    rclpy.init(args=args)
    node = SwerveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
