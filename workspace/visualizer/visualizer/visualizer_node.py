#!/usr/bin/env python3
import os
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
from ament_index_python.packages import get_package_share_directory

########
import rclpy
import rclpy.logging
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from rclpy.duration import Duration

from project_utils.msg import Eigen2dVector, EigenVector


pkg_share = get_package_share_directory("project_utils")
sim_info_path = os.path.join(pkg_share,'config','sim_config.txt')
sim_info  = np.loadtxt(sim_info_path)

#print(matplotlib.get_backend())  # Should output "QtAgg"
# matplotlib.use("Agg")  # Explicitly set the backend
import matplotlib.pyplot as plt
import matplotlib.lines as lines
import matplotlib.patches as patches


# from tf2.transformations import euler_from_quaternion
import threading

import pdb
#####################
class visualizer_class(Node):
    def __init__(self):
        super().__init__("visualizer_node")
        self.node_init()
        self.state_sub  = self.create_subscription(Eigen2dVector,'/states',self.state_sub_callback,10)
        self.state_sub
        self.timer = self.create_timer(self.plot_update_dt,self.plot_cb)
        self.plot_lock = threading.Lock()
        

    def node_init(self):
        self.plot_update_dt = 0.1
        self.paused = False
        self.num_drones = int(sim_info[0][0])
        self.num_obs = int(sim_info[0][1])
        self.dt = float(sim_info[0][2])
        self.dim_drone = sim_info[1]
        self.a_drone,self.b_drone,self.c_drone  = self.dim_drone
        self.init_drone_pose = sim_info[2:2+self.num_drones]
        self.goal_drone_pose  = sim_info[2+self.num_drones:2+2*self.num_drones]
        self.pose_obs = sim_info[2+2*self.num_drones:2+2*self.num_drones+self.num_obs]
        self.dim_obs = sim_info[2+2*self.num_drones+self.num_obs:]


        self.x_obs = self.pose_obs[:,0]
        self.y_obs = self.pose_obs[:,1]
        self.z_obs = self.pose_obs[:,2]        
        self.a_obs = self.dim_obs[:,0]
        self.b_obs = self.dim_obs[:,1]
        self.c_obs = self.dim_obs[:,2]
        
        self.x_lim = [-2,2]
        self.y_lim = [-2,2]
        self.z_lim = [+0.2,2.2]
        
        
        self.robot_fig , self.robot_axis = plt.subplots(subplot_kw={'projection': '3d'})
        self.robot_axis.set_xlim( self.x_lim[0], self.x_lim[1]  )
        self.robot_axis.set_ylim( self.y_lim[0] , self.y_lim[1] )
        self.robot_axis.set_zlim( self.z_lim[0] , self.z_lim[1] )
        
        self.robot_axis.set_title('Trajectory')
        self.robot_axis.set_xlabel('x in m')
        self.robot_axis.set_ylabel('y in m')
        self.robot_axis.set_zlabel('z in m')
        self.robot_axis.view_init(elev=60, azim=-77)
        
        self.phi_obs = np.linspace(0,2*np.pi,10).reshape(10,1)  #azimuthal angle rotoation around z axis
        self.theta_obs = np.linspace(0,np.pi/2,10).reshape(-1,10)  # tilt from top pole (z-axis)
        
        self.phi_drone = np.linspace(0,2*np.pi,10).reshape(10,1)
        self.theta_drone = np.linspace(0,np.pi,10).reshape(-1,10)
        
        self.colors = (np.random.choice(range(255),size=[self.num_drones,3]))/255.0
        
        if self.num_obs  > 0:
            for k in range(len(self.x_obs)):
                if self.c_obs[k] > self.z_lim[1]:
                    self.z = np.linspace(self.z_obs[k],2.0,15)
                    self.theta = np.linspace(0,2*np.pi,15)
                    self.theta_obs, self.z_ell_obs  = np.meshgrid(self.theta,self.z)
                    
                    self.x_ell_obs  = self.x_obs[k] + self.a_obs[k]*np.cos(self.theta_obs)
                    self.y_ell_obs =  self.y_obs[k] + self.b_obs[k]*np.sin(self.theta_obs)
                else:
                    self.x_ell_obs = self.x_obs[k] + self.a_obs[k]*np.sin(self.theta_obs)*np.cos(self.phi_obs)
                    self.y_ell_obs = self.y_obs[k] + self.b_obs[k]*np.sin(self.theta_obs)*np.sin(self.phi_obs)
                    self.z_ell_obs = self.z_obs[k] + self.c_obs[k]*np.cos(self.theta_obs)
                self.robot_axis.plot_surface(self.x_ell_obs,self.y_ell_obs,self.z_ell_obs,rstride=10, cstride=2, color='#2980b9', alpha=0.4)
        
        self.robot_axis.plot(
                             [self.x_lim[0] ,  self.x_lim[0] , self.x_lim[1] , self.x_lim[1] , self.x_lim[0]],\
                             [self.y_lim[0] , self.y_lim[1] , self.y_lim[1] , self.y_lim[0], self.y_lim[0]],
                             color = 'red',
                             alpha = 0.1
                             )
        
        self.robot_axis.plot(
                        [self.x_lim[0] ,  self.x_lim[0] , self.x_lim[1] , self.x_lim[1], self.x_lim[0]],\
                        [self.y_lim[0] , self.y_lim[1] , self.y_lim[1] , self.y_lim[0] , self.y_lim[0]],
                        [self.z_lim[1] , self.z_lim[1] , self.z_lim[1] , self.z_lim[1] , self.z_lim[1]],
                        color = 'red',
                        alpha = 0.1
                        )    
        
        self.robot_axis.plot([self.x_lim[0], self.x_lim[0]], [self.y_lim[0], self.y_lim[0]], [self.z_lim[0], self.z_lim[1]], color='red', alpha=0.6)
        self.robot_axis.plot([self.x_lim[0], self.x_lim[0]], [self.y_lim[1], self.y_lim[1]], [self.z_lim[0], self.z_lim[1]], color='red', alpha=0.6)
        self.robot_axis.plot([self.x_lim[1], self.x_lim[1]], [self.y_lim[1], self.y_lim[1]], [self.z_lim[0], self.z_lim[1]], color='red', alpha=0.6)
        self.robot_axis.plot([self.x_lim[1], self.x_lim[1]], [self.y_lim[0], self.y_lim[0]], [self.z_lim[0], self.z_lim[1]], color='red', alpha=0.6)
        
        self.collision_count_agent  = 0
        self.collision_count_obs = 0
        #self.sim_steps = (len(self.sim_data)/self.num_drone/3)    
        
        self.drones_surfaces = [] # only store one reference for N drones
        self.drone_poses = []
        self.stats = None
        self.update_poses_init()
        
    def update_poses_init(self):
        self.drone_poses = []
        rand = np.random.random_sample(size=(self.num_drones, 3))

        # Scale for each axis using: scaled = min + (max - min) * rand
        poses = np.empty_like(rand)
        poses[:, 0] = self.x_lim[0] + (self.x_lim[1] - self.x_lim[0]) * rand[:, 0]  # x
        poses[:, 1] = self.y_lim[0] + (self.y_lim[1] - self.y_lim[0]) * rand[:, 1]  # y
        poses[:, 2] = self.z_lim[0] + (self.z_lim[1] - self.z_lim[0]) * rand[:, 2]  # z

        self.drone_poses = poses
        
        
    def update_poses(self,poses):
        self.drone_poses = []
        #self.drone_poses = poses
        for i in range(0,self.num_drones):
            self.drone_poses.append([poses[i].data[0], poses[i].data[1] ,poses[i].data[2] ])
            
        
        

    
    def update_ellipsoids(self,poses,a_drone,b_drone,c_drone):
        for surface in self.drones_surfaces:
            surface.remove()
        if self.stats != None:
            self.stats.remove()
        self.drones_surfaces = []
        for k in range(0,self.num_drones):
            x_ell_drone = poses[k,0] + a_drone*np.sin(self.theta_drone)*np.cos(self.phi_drone)
            y_ell_drone = poses[k,1] + b_drone*np.sin(self.theta_drone)*np.sin(self.phi_drone)
            z_ell_drone = poses[k,2] + c_drone*np.cos(self.theta_drone)
            body_temp = self.robot_axis.plot_surface(x_ell_drone, y_ell_drone, z_ell_drone,  rstride=4, cstride=6, color = self.colors[k], alpha=0.6)
            self.drones_surfaces.append(body_temp)
        
        for i in range(0,self.num_drones):
            x_ego = poses[i,0]
            y_ego = poses[i,1]
            z_ego = poses[i,2]
            for j in range(0,self.num_drones):
                if i==j:
                    continue
                val = (x_ego- poses[j,0])**2 / (2*self.a_drone)**2 + (y_ego- poses[j,1])**2 / (2*self.b_drone)**2 + (z_ego- poses[j,2])**2 / (2*self.c_drone)**2
                if val<1:
                    self.collision_count_agent+=1
            if self.num_obs > 0:
                for k in range(0,len(self.a_obs)):
                    val = (x_ego - self.x_obs[j])**2 / (self.a_obs[k]+self.a_drone)**2 + (y_ego- self.y_obs[j])**2 / (self.b_obs[k]+self.b_drone)**2 + (z_ego- self.z_obs[j])**2 / (self.c_obs[j]+self.c_drone)**2
                    if val<1:
                        self.collision_count_obs += 1
    
        self.stats = self.robot_axis.text(0, self.y_lim[1], self.z_lim[1],'Obstacle Collision Count = {obs}\nInter-Agent Collision Count = {col}\nAgents = {n}\nSim-Step = {step}'.format(obs=self.collision_count_obs, col=self.collision_count_agent, n=self.num_drones, step=i)) 
    
    def state_sub_callback(self,msg):
        
        poses =  msg.data   # NP array
        self.update_poses(poses)

    
    def plot_cb(self):
        poses = self.drone_poses
        self.update_ellipsoids(poses,self.a_drone,self.b_drone,self.c_drone)
        # self.ego_x_history.append(self.ego_x_pose)
        # self.ego_y_history.append(self.ego_y_pose)
   
   
    def set_lines(self,col,waypoints, ax , fig,line_size=1 ):
        track_line =  lines.Line2D([],[],linestyle='--',color = col ,linewidth=line_size)
        track_line.set_data(waypoints[:,0],waypoints[:,1])
        ax.add_line(track_line)  
        
                
    def spin(self):
        plt.ion()
        self.robot_fig.show()
        self.get_logger().info("vehicle_sim_node spinning")
        # plt.show()
        rclpy.spin(self)
        plt.close(self.robot_fig)

def main():
    #plt.switch_backend("Qt4Agg")
    rclpy.init()
    visualizer_class_node = visualizer_class()
    visualizer_class_node.spin()
    rclpy.shutdown()

if __name__=='__main__':
    main()