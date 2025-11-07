#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time

node = None
turtle1_pose = None
cmd_pub = None

def pose_callback(msg):
    global turtle1_pose
    turtle1_pose = msg

rclpy.init()
node = Node('test_movement')
node.create_subscription(Pose, '/turtle1/pose', pose_callback, 10)
cmd_pub = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

print("Waiting for pose...")
while turtle1_pose is None:
    rclpy.spin_once(node, timeout_sec=0.1)

print(f"Start: ({turtle1_pose.x:.2f}, {turtle1_pose.y:.2f}), theta={math.degrees(turtle1_pose.theta):.1f}°")

# Target
target_x, target_y = 8.0, 8.0
print(f"Target: ({target_x}, {target_y})")

# Test movement with gains 2.0, 4.0
linear_gain = 2.0
angular_gain = 4.0

for i in range(50):
    rclpy.spin_once(node, timeout_sec=0.01)
    
    dx = target_x - turtle1_pose.x
    dy = target_y - turtle1_pose.y
    distance = math.sqrt(dx*dx + dy*dy)
    
    desired_angle = math.atan2(dy, dx)
    angle_diff = desired_angle - turtle1_pose.theta
    while angle_diff > math.pi: angle_diff -= 2*math.pi
    while angle_diff < -math.pi: angle_diff += 2*math.pi
    
    if i % 10 == 0:
        print(f"[{i}] dist={distance:.3f}, angle={angle_diff:.3f} ({math.degrees(angle_diff):.1f}°)")
        print(f"     cmd: vx={linear_gain*distance:.3f}, wz={angular_gain*angle_diff:.3f}")
    
    if distance < 0.5:
        print("✅ Reached target!")
        break
    
    msg = Twist()
    msg.linear.x = linear_gain * distance
    msg.angular.z = angular_gain * angle_diff
    cmd_pub.publish(msg)
    time.sleep(0.2)

# Stop
msg = Twist()
cmd_pub.publish(msg)

print(f"End: ({turtle1_pose.x:.2f}, {turtle1_pose.y:.2f})")
node.destroy_node()
rclpy.shutdown()
