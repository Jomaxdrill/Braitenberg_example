#!/usr/bin/env python3

#line above indicate it’s a Python script
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Turtlebot3 Burger parameters
MAX_SPEED = 0.22  # TurtleBot3 Burger max linear speed
SCALE = 1.0 / 3.5  # Normalize to LIDAR range (max 3.5m)
WHEEL_DIST = 0.16 # Wheel separation ~0.16m
BEHAVIOR = {0: 'fearful', 1: 'aggressive'} #Vehicle 2A and Vehicle 2B

class BraitenbergVehicle(Node):
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('braitenberg_vehicle')

        # Declare parameters with default values
        self.declare_parameter('behavior', BEHAVIOR[0])  # Default: 'fearful'
        self.declare_parameter('speed', MAX_SPEED)       # Default: max speed

        # Publisher for velocity commands
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for LIDAR data
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # Velocity message
        self.twist = Twist()

        # Log that the node has started
        self.get_logger().info('Braitenberg vehicle node started')

    def scan_callback(self, data):
        # Get parameter values
        speed = float(self.get_parameter('speed').value)
        behavior_value = self.get_parameter('behavior').value
        behavior = behavior_value if behavior_value in BEHAVIOR.values() else BEHAVIOR[0]

        # Extract LIDAR ranges (360 degrees, 0 = forward, 90 = left, 270 = right)
        ranges = data.ranges
        # Define left and right sensor zones (indices for 360 samples)
        left_idx = range(45, 135)   # ~45° to 135° (left side)
        right_idx = range(225, 315) # ~225° to 315° (right side)

        # Average distances for left and right (ignore inf/nan)
        left_dist = min([r for r in [ranges[i] for i in left_idx] if r < 3.5], default=3.5)
        right_dist = min([r for r in [ranges[j] for j in right_idx] if r < 3.5], default=3.5)
        self.get_logger().info(f'behavior: {behavior}, Left dist: {left_dist}, Right dist: {right_dist}')

        if behavior == 'fearful':
            # Vehicle 2a: Turn away from obstacles
            left_speed = speed * (1 - SCALE * left_dist)
            right_speed = speed * (1 - SCALE * right_dist)
        elif behavior == 'aggressive':
            # # Vehicle 2b: Stronger sensor input -> faster wheel on opposite side -> turn toward
            left_speed = speed * (1 - SCALE * right_dist)
            right_speed = speed * (1 - SCALE * left_dist)

        self.twist.linear.x = (left_speed + right_speed) / 2
        self.twist.angular.z = (right_speed - left_speed) / WHEEL_DIST
        # Publish velocity
        self.vel_pub.publish(self.twist)

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create the node
    vehicle = BraitenbergVehicle()

    try:
        # Keep the node running
        rclpy.spin(vehicle)
    except KeyboardInterrupt:
        # Handle shutdown gracefully
        vehicle.get_logger().info('Shutting down Braitenberg vehicle node')
    finally:
        # Cleanup
        vehicle.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()