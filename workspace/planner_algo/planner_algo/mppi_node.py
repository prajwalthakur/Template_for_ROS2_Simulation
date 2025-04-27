#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from project_utils.msg import EigenVector , Pj
from transforms3d.euler import quat2euler,euler2quat

class mppiNode(Node):
    def __init__(self):
        super().__init__("mppi_node")

        self.control_pub = self.create_publisher(EigenVector, '/control', 10)
        self.state_sub = self.create_subscription(PoseStamped, '/robot_state', self.state_sub_callback,10)

        #TODO: 0.05 should match the dynamics-update self.dt 
        self.control_update_dt = self.create_timer(0.05, self.mppi_call)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def mppi_call(self):
        x= self.x
        
        
        randommsg = Pj()
        randommsg.x = 76.0
        randommsg.y = 76.0
        
        msg = EigenVector()
        msg.data =  [0.0, 0.0]
        self.control_pub.publish(msg)
        
    def state_sub_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        _,_, self.yaw = quat2euler([qw, qx, qy, qz])

    def update_states(self):
        self.robot.step()
        pose_msg = PoseStamped()
        pose = self.robot.get_states().reshape(self.robot.num_state)
        pose_msg.pose.position.x = float(pose[0])
        pose_msg.pose.position.y = float(pose[1])
        qw, qx, qy, qz = euler2quat(0, 0, pose[2])  # roll=0, pitch=0, yaw=pose[2]
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz #np.sin(pose[2] / 2.0)
        pose_msg.pose.orientation.w = qw #np.cos(pose[2] / 2.0)
        self.state_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = mppiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
