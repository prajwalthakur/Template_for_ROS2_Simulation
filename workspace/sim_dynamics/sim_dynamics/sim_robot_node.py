#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as lines
import threading
import time
from sim_dynamics.robot_dynamics import robot_dynamics
from transforms3d.euler import quat2euler
from transforms3d.euler import euler2quat

class VisualizerNode(Node):
    def __init__(self):
        super().__init__("visualizer_node")
        self.pose_sub = self.create_subscription(PoseStamped, '/robot_state', self.pose_callback, 10)
        self.robot = robot_dynamics()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.robot_r =  self.robot.wheel_base/2.0  # robot radius

        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-15, 15)
        self.ax.set_ylim(-15, 15)
        self.ax.set_title('Robot ')
        self.ax.set_aspect('equal')
        
        self.robot_patch = patches.Circle((self.x, self.y), radius=self.robot_r, fc='blue', alpha=0.5)
        self.heading_line = lines.Line2D([], [], color='black', linewidth=2)
        self.ax.add_patch(self.robot_patch)
        self.ax.add_line(self.heading_line)
        self.ax.grid(True)

    def pose_callback(self, msg):
        
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        _,_, self.yaw = quat2euler([qw, qx, qy, qz])

        # Update plot immediately
        self.robot_patch.center = (self.x, self.y)
        
        end_x = self.x + self.robot_r * np.cos(self.yaw)
        end_y = self.y + self.robot_r * np.sin(self.yaw)
        
        self.heading_line.set_data([self.x, end_x], [self.y, end_y])

        #
        self.fig.canvas.draw_idle()  # Just request redraw (non-blocking)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()

    plt.ion()
    node.fig.show()

    # New: Keep running until user closes the plot
    try:
        while plt.fignum_exists(node.fig.number):
            rclpy.spin_once(node, timeout_sec=0.1)  # Spin only a little at a time
            node.fig.canvas.flush_events()
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    plt.close(node.fig)


if __name__ == '__main__':
    main()
