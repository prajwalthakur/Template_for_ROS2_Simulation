#!/usr/bin/env python3
import numpy as np
class robot_dynamics():
    def __init__(self):
        self.vmax_l = 2
        self.vmin_l = -2
        self.vmax_r = 2
        self.vmin_r = -2
        self.omega_max = 0.4
        self.omega_min = -0.4
        self.wheel_base = 3
        self.num_state = 3
        self.num_control = 2
        #TODO: update the self.dt 
        self.dt = 0.01
        self.control = np.zeros((self.num_control, 1))
        self.states = np.zeros((self.num_state, 1))

    def ode(self, st, ut):
        vr = ut[0][0]
        vl = ut[1][0]   
        x = st[0][0]
        y = st[1][0]
        delta = st[2][0]
        x_dot = (vr + vl) * np.cos(delta) / 2
        y_dot = (vr + vl) * np.sin(delta) / 2
        delta_dot = (vr - vl) / self.wheel_base
        return np.array([x_dot, y_dot, delta_dot]).reshape(self.num_state, 1)

    def step(self):
        st = self.states
        ut = self.get_control()
        k1 = self.ode(st, ut)
        k2 = self.ode(st + (self.dt/2.0)*k1, ut)
        k3 = self.ode(st + (self.dt/2.0)*k2, ut)
        k4 = self.ode(st + (self.dt/2.0)*k3, ut)
        x_next = st + self.dt * (k1/6. + k2/3. + k3/3. + k4/6.)
        self.states = x_next

    def get_control(self):
        return self.control

    def get_states(self):
        return self.states

    def set_control(self, control):
        self.control = control

    def set_states(self, states):
        self.states = states
        
    def set_dt(self,dt):
        self.dt = dt