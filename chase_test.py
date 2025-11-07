#!/usr/bin/env python3
import math, rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

turtle1_pose = None
target_pose = None

def turtle1_cb(msg):
    global turtle1_pose
    turtle1_pose = msg

def target_cb(msg):
    global target_pose
    target_pose = msg

rclpy.init()
node = Node('chase_test')
node.create_subscription(Pose, '/turtle1/pose', turtle1_cb, 10)
node.create_subscription(Pose, '/target/pose', target_cb, 10)
cmd_pub = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

print("Waiting for poses...")
while turtle1_pose is None or target_pose is None:
    rclpy.spin_once(node, timeout_sec=0.1)

print("CHASE TEST with gains 5.0/6.0")
print(f"turtle1: ({turtle1_pose.x:.2f}, {turtle1_pose.y:.2f})")
print(f"target: ({target_pose.x:.2f}, {target_pose.y:.2f}) - MOVING")

linear_gain = 5.0
angular_gain = 6.0

for i in range(100):
    rclpy.spin_once(node, timeout_sec=0.01)
    
    dx = target_pose.x - turtle1_pose.x
    dy = target_pose.y - turtle1_pose.y
    dist = math.sqrt(dx*dx + dy*dy)
    
    desired = math.atan2(dy, dx)
    angle = desired - turtle1_pose.theta
    while angle > math.pi: angle -= 2*math.pi
    while angle < -math.pi: angle += 2*math.pi
    
    if i % 20 == 0:
        print(f"[{i}] dist={dist:.2f}, angle={math.degrees(angle):.0f}°, vx={linear_gain*dist:.2f}, wz={angular_gain*angle:.2f}")
    
    if dist < 0.5:
        print("✅ CAUGHT!")
        break
    
    msg = Twist()
    msg.linear.x = linear_gain * dist
    msg.angular.z = angular_gain * angle
    cmd_pub.publish(msg)
    time.sleep(0.1)

cmd_pub.publish(Twist())
print(f"Final distance: {dist:.2f}")
node.destroy_node()
rclpy.shutdown()
