#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

count = 0

def pose_cb(msg):
    global count
    count += 1
    if count % 10 == 0:
        print(f"[{count}] turtle1 pose: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}")

rclpy.init()
node = Node('pose_test')
node.create_subscription(Pose, '/turtle1/pose', pose_cb, 10)

print("Listening to /turtle1/pose...")
print("Publish to /turtle1/cmd_vel to make it move")

for i in range(100):
    rclpy.spin_once(node, timeout_sec=0.1)

print(f"Total callbacks received: {count}")
node.destroy_node()
rclpy.shutdown()
