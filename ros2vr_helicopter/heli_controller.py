from ros2vr_interface.msg import VRobotStates, VRobotCMD, Collision
from ubicoders_vrobots.vrobots_msgs.VROBOTS_CMDS import VROBOTS_CMDS
import numpy as np

class PIController:
    def __init__(self, kp=1, ki=1, error_max=100):
        self.kp = kp
        self.ki = ki
        self.integral = 0
        self.error_max = error_max

    def error_integrator(self, error):
        self.integral += error
        self.integral = np.clip(self.integral, -self.error_max, self.error_max)
    
    def update(self, error):
        self.error_integrator(error)
        return self.kp * error + self.ki * self.integral


class HeliController():
    def __init__(self) -> None:
        # z axis pos, vel control
        self.body_z_pos_ctrl = PIController(kp=1.1, ki=0.5, error_max=5)
        self.body_z_vel_ctrl = PIController(kp=0.75, ki=0.2, error_max=10)


        # x axis control (attitude and rate)
        self.body_x_att_ctrl = PIController(kp=1, ki=0, error_max=1)
        self.body_x_rate_ctrl = PIController(kp=0.5, ki=0, error_max=10)

        # z axis control (attitude and rate)
        self.body_z_att_ctrl = PIController(kp=5*10**-3, ki=10**-4, error_max=1)
        self.body_z_rate_ctrl = PIController(kp=120, ki=1.25, error_max=10)


    def update(self, statesMsg:VRobotStates):
        main_coll  = 4.5
        cyclic_pitch = 0.0
        cyclic_azim = 0.0
        #coll_tail = 0

        # altitude control
        body_z_pos_setpoint = -10# -100
        body_z_pos_error = body_z_pos_setpoint - statesMsg.lin_pos.z
        body_z_vel_setpoint = self.body_z_pos_ctrl.update(body_z_pos_error)
        body_z_vel_setpoint = np.clip(body_z_vel_setpoint, -7.0, 7.0)

        # altitude rate control
        # body_z_vel_setpoint = -10
        body_z_vel_error = body_z_vel_setpoint - statesMsg.lin_vel.z
        main_coll = self.body_z_vel_ctrl.update(-body_z_vel_error)
        main_coll = np.clip(main_coll, 0.0, 7.0)
        print(main_coll)

        # body_x_att control
        body_x_att_error = 0 - statesMsg.euler.x
        body_x_rate_setpoint = self.body_x_att_ctrl.update(body_x_att_error)
        body_x_rate_setpoint = np.clip(body_x_rate_setpoint, 0, 1.0)

        # body_x_rate control
        body_x_rate_error = (body_x_rate_setpoint - statesMsg.ang_vel.x)
        cyclic_pitch = self.body_x_rate_ctrl.update(body_x_rate_error)
        print(cyclic_pitch)
        cyclic_pitch = np.clip(cyclic_pitch, 0, 1.0)
        cyclic_azim = 0.0


        # body_z_att control
        body_z_att_error = 0 - statesMsg.euler.z 
        body_z_rate_setpoint = self.body_z_att_ctrl.update(body_z_att_error)
        
        # body_z_rate control
        body_z_rate_error = (body_z_rate_setpoint - statesMsg.ang_vel.z)
        coll_tail = self.body_z_rate_ctrl.update(-body_z_rate_error)
        coll_tail = np.clip(coll_tail, -4.0, 4.0)
        #print(coll_tail)
        

        # coll_main, cyclic_pitch, cyclic_azim, coll_tail (deg)
        return [main_coll, # collective pitch angle - main rotor (deg)
                np.clip(cyclic_pitch, -1.0, 1.0),
                np.clip(cyclic_azim, 0.0, 360.0),
                coll_tail,
                ]