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
        # NOTE: Topic names must match your controllers.yaml
        self.pivot_pub = self.create_publisher(Float64MultiArray, '/pivot_controller/commands', 10)
        self.drive_pub = self.create_publisher(Float64MultiArray, '/drive_controller/commands', 10)
        
        # Robot Dimensions (Must match URDF)
        self.L = 0.6  # Wheelbase Length
        self.W = 0.4  # Track Width

        self.get_logger().info("Swerve Controller Started - Ready to Drive!")

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
        half_L = self.L / 2.0
        half_W = self.W / 2.0
        
        # 1. Front Left (+X, +Y)
        v_fl_x = vx - w * half_W
        v_fl_y = vy + w * half_L
        
        # 2. Front Right (+X, -Y)
        v_fr_x = vx - w * (-half_W) # equivalent to vx + w*half_W
        v_fr_y = vy + w * half_L

        # 3. Back Left (-X, +Y)
        v_bl_x = vx - w * half_W
        v_bl_y = vy + w * (-half_L) # equivalent to vy - w*half_L

        # 4. Back Right (-X, -Y)
        v_br_x = vx - w * (-half_W) # equivalent to vx + w*half_W
        v_br_y = vy + w * (-half_L) # equivalent to vy - w*half_L

        # --- POLAR COORDINATES (Speed & Angle) ---
        
        # Front Left
        speed_fl = math.hypot(v_fl_x, v_fl_y)
        angle_fl = math.atan2(v_fl_y, v_fl_x)
        
        # Front Right
        speed_fr = math.hypot(v_fr_x, v_fr_y)
        angle_fr = math.atan2(v_fr_y, v_fr_x)
        
        # Back Left
        speed_bl = math.hypot(v_bl_x, v_bl_y)
        angle_bl = math.atan2(v_bl_y, v_bl_x)
        
        # Back Right
        speed_br = math.hypot(v_br_x, v_br_y)
        angle_br = math.atan2(v_br_y, v_br_x)

        # --- PUBLISH COMMANDS ---
        # CRITICAL: Order must be ALPHABETICAL [BL, BR, FL, FR] 
        # because ROS controllers sort joint names alphabetically.

        pivot_msg = Float64MultiArray()
        pivot_msg.data = [angle_bl, angle_br, angle_fl, angle_fr]
        self.pivot_pub.publish(pivot_msg)

        drive_msg = Float64MultiArray()
        drive_msg.data = [speed_bl, speed_br, speed_fl, speed_fr]
        self.drive_pub.publish(drive_msg)

    def stop_robot(self):
        # Stop driving, but keep wheels at current angle (don't snap to 0)
        d_msg = Float64MultiArray()
        d_msg.data = [0.0, 0.0, 0.0, 0.0] # Stop motors
        self.drive_pub.publish(d_msg)
        # We purposely do not publish to pivot_pub here.

def main(args=None):
    rclpy.init(args=args)
    node = SwerveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
