#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


from project_utils.msg import EigenVector

from sim_dynamics.robot_dynamics import robot_dynamics
from transforms3d.euler import euler2quat
class DynamicsNode(Node):
    def __init__(self):
        super().__init__("update_dynamics_node")
        self.robot = robot_dynamics()
        self.robot.set_states(np.zeros((self.robot.num_state, 1)))
        
        self.state_pub = self.create_publisher(PoseStamped, '/robot_state', 10)
        self.control_sub = self.create_subscription(EigenVector, '/control', self.control_sub_callback, 10)

        #TODO: 0.01 should match the dynamics-update self.dt 
        self.state_update_timer = self.create_timer(0.01, self.update_states)
        self.robot.set_dt(dt = 0.01)

    def control_sub_callback(self, msg):
        control = msg.data
        control = np.array(control)
        control = control.reshape(self.robot.num_control, 1)
        self.robot.set_control(control)

    def update_states(self):
        self.robot.step()
        
        # to publish the states
        pose_msg = PoseStamped()
        pose = self.robot.get_states().reshape(self.robot.num_state) #
        pose_msg.pose.position.x = float(pose[0])
        pose_msg.pose.position.y = float(pose[1])
        qw, qx, qy, qz = euler2quat(0, 0, pose[2])  
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz #np.sin(pose[2] / 2.0)
        pose_msg.pose.orientation.w = qw #np.cos(pose[2] / 2.0)
        
        self.state_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
