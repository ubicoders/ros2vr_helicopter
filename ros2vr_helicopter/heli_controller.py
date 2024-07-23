from ros2vr_interface.msg import VRobotStates, VRobotCMD, Collision
from ubicoders_vrobots.vrobots_msgs.VROBOTS_CMDS import VROBOTS_CMDS
import numpy as np

class PIDController:
    def __init__(self, kp=1, kd=1, ki=1, error_max=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.error_max = error_max

    def error_integrator(self, error):
        self.integral += error
        self.integral = np.clip(self.integral, -self.error_max, self.error_max)
    
    def update(self, error, d_error=0):
        self.error_integrator(error)
        return self.kp * error + self.ki * self.integral + d_error * self.kd


class HeliController():
    def __init__(self) -> None:
        # z axis pos, vel control
        self.body_z_pos_ctrl = PIDController(kp=2.1, ki=0.25, error_max=10)
        self.body_z_vel_ctrl = PIDController(kp=0.75, ki=0.2, error_max=10)

        # x axis pos, vel control
        self.body_x_pos_ctrl = PIDController(kp=0.1, kd=1, ki=0.0, error_max=10)
        self.body_y_pos_ctrl = PIDController(kp=2,   kd=1,  ki=0.1, error_max=10)

        # x-y axis control (attitude and rate)
        self.body_x_att_ctrl = PIDController(kp=1.5, kd=30, ki=0, error_max=10)
        self.body_y_att_ctrl = PIDController(kp=1.5, kd=20, ki=0, error_max=10)

        # z axis control (attitude and rate)
        self.body_z_att_ctrl = PIDController(kp=5*10**-3, ki=10**-4, error_max=1)
        self.body_z_rate_ctrl = PIDController(kp=120, ki=1.25, error_max=10)


    def update(self, statesMsg:VRobotStates):
        main_coll  = 4.5
        cyclic_pitch = 0.0
        cyclic_azim = 0.0
        #coll_tail = 0

        #=======================================================================
        # Altitude Controllers
        #=======================================================================
        # altitude control
        body_z_pos_setpoint = -20# -100
        body_z_pos_error = body_z_pos_setpoint - statesMsg.lin_pos.z
        body_z_vel_setpoint = self.body_z_pos_ctrl.update(body_z_pos_error)
        body_z_vel_setpoint = np.clip(body_z_vel_setpoint, -7.0, 7.0)

        # altitude rate control
        # body_z_vel_setpoint = -10
        body_z_vel_error = body_z_vel_setpoint - statesMsg.lin_vel.z
        main_coll = self.body_z_vel_ctrl.update(-body_z_vel_error)
        main_coll = np.clip(main_coll, 0.0, 7.0)
        #print(main_coll)

        # =======================================================================
        # Position XY Controllers
        # =======================================================================
        body_x_pos_setpoint = 100
        body_x_pos_error = body_x_pos_setpoint - statesMsg.lin_pos.x
        body_x_vel_setpoint = self.body_x_pos_ctrl.update(body_x_pos_error, -statesMsg.lin_vel.x)
        print(f"body_x_vel_setpoint: {body_x_vel_setpoint}")
        att_y_setpoint = np.clip(-body_x_vel_setpoint, -10.0, 10.0)

        body_y_pos_setpoint = 160.01
        body_y_pos_error = body_y_pos_setpoint - statesMsg.lin_pos.y
        body_y_vel_setpoint = self.body_y_pos_ctrl.update(body_y_pos_error, -statesMsg.lin_vel.y)
        print(f"body_y_vel_setpoint: {body_y_vel_setpoint}")
        att_x_setpoint = np.clip(body_y_vel_setpoint, -10.0, 10.0)


        #=======================================================================
        # Attitude Controllers
        #=======================================================================
        # body_x_att control
        body_x_att_error = att_x_setpoint - statesMsg.euler.x
        body_x_rate_setpoint = self.body_x_att_ctrl.update(body_x_att_error, -statesMsg.ang_vel.x)
        body_x_rate_setpoint = np.clip(body_x_rate_setpoint, -1.0, 1.0)

        # body_y_att control
        body_y_att_error = att_y_setpoint - statesMsg.euler.y
        body_y_rate_setpoint = self.body_y_att_ctrl.update(body_y_att_error, -statesMsg.ang_vel.y)
        body_y_rate_setpoint = np.clip(body_y_rate_setpoint, -1.0, 1.0)
        
        #=======================================================================
        # Rate Controllers
        #=======================================================================
        # calculate cyclic pitch and cyclic azimuth
        cyclic_vector = np.array([-body_x_rate_setpoint, body_y_rate_setpoint])
        mag_cyclic_vector = np.linalg.norm(cyclic_vector)
        unit_cyclic_vector = cyclic_vector / mag_cyclic_vector if mag_cyclic_vector > 0 else cyclic_vector
        disk_top_azim = np.arctan2(unit_cyclic_vector[1], unit_cyclic_vector[0]) * 180 / np.pi
        disk_top_azim = convert_to_360_range(convert_to_360_range(disk_top_azim)+90)
        cyclic_azim = convert_to_360_range(disk_top_azim - 90)
        cyclic_pitch = mag_cyclic_vector * 0.25
        

        # body_z_att control
        body_z_att_error = 0 - statesMsg.euler.z 
        body_z_rate_setpoint = self.body_z_att_ctrl.update(body_z_att_error)
        body_z_rate_setpoint = np.clip(body_z_rate_setpoint, -1.0, 1.0)
        
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
    
def convert_to_360_range(value):
    if value < 0:
        value += 360
    return value % 360    