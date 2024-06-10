import math
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

from project.control.BaseControl import BaseControl
from gym_pybullet_drones.utils.enums import DroneModel

class Vel_AngVel_PIDControl(BaseControl):
    """PID control class for Crazyflies.

    Basato su un controllore di velocità PID

    """

    ################################################################################

    def __init__(self,
                 drone_model: DroneModel,
                 g: float=9.8
                 ):
        """Common control classes __init__ method.

        Parameters
        ----------
        drone_model : DroneModel
            The type of drone to control (detailed in an .urdf file in folder `assets`).
        g : float, optional
            The gravitational acceleration in m/s^2.

        """
        super().__init__(drone_model=drone_model, g=g)
        if self.DRONE_MODEL != DroneModel.CF2X and self.DRONE_MODEL != DroneModel.CF2P:
            print("[ERROR] in DSLPIDControl.__init__(), DSLPIDControl requires DroneModel.CF2X or DroneModel.CF2P")
            exit()
        self.P_COEFF_VEL = np.array([.4, .4, 1.25])
        self.I_COEFF_VEL = np.array([.05, .05, .05])
        self.D_COEFF_VEL = np.array([.2, .2, .5])
        self.P_COEFF_TOR = np.array([70000., 70000., 60000.])
        self.I_COEFF_TOR = np.array([.0, .0, 500.])
        self.D_COEFF_TOR = np.array([20000., 20000., 12000.])
        self.P_COEFF_ANGVEL = np.array([70000., 70000., 60000.])
        self.I_COEFF_ANGVEL = np.array([.0, .0, 500.])
        self.D_COEFF_ANGVEL = np.array([20000., 20000., 12000.])
        self.PWM2RPM_SCALE = 0.2685
        self.PWM2RPM_CONST = 4070.3
        self.MIN_PWM = 20000
        self.MAX_PWM = 65535

        self.integral_vel_e = np.zeros(3)
        self.prev_vel_e = np.zeros(3)  # Per memorizzare l'errore precedente
        
        if self.DRONE_MODEL == DroneModel.CF2X:
            self.MIXER_MATRIX = np.array([ 
                                    [-.5, -.5, -1],
                                    [-.5,  .5,  1],
                                    [.5, .5, -1],
                                    [.5, -.5,  1]
                                    ])
        elif self.DRONE_MODEL == DroneModel.CF2P:
            self.MIXER_MATRIX = np.array([
                                    [0, -1,  -1],
                                    [+1, 0, 1],
                                    [0,  1,  -1],
                                    [-1, 0, 1]
                                    ])
        self.reset()

    ################################################################################

    def reset(self):
        """Resets the control classes.

        The previous step's and integral errors for both position and attitude are set to zero.

        """
        super().reset()
        
        #### Store the last roll, pitch, and yaw ###################
        self.last_rpy = np.zeros(3)
        #### Store the last roll_rate, pitch_rate, and yaw_rate ###################
        self.last_rpy_rates = np.zeros(3)
        #### Initialized PID control variables #####################
        self.last_pos_e = np.zeros(3)
        self.integral_pos_e = np.zeros(3)
        self.last_rpy_e = np.zeros(3)
        self.integral_rpy_e = np.zeros(3)

        self.last_vel_e = np.zeros(3)
        self.integral_vel_e = np.zeros(3)
        self.last_rpy_rates_e = np.zeros(3)
        self.integral_rpy_rates_e = np.zeros(3)

    ################################################################################
    
    def computeControl(self,
                       control_timestep,
                       cur_pos,
                       cur_quat,
                       cur_vel,
                       cur_ang_vel,
                       target_pos,
                       target_rpy=np.zeros(3),
                       target_vel=np.zeros(3),
                       target_rpy_rates=np.zeros(3)
                       ):
        """Computes the PID control action (as RPMs) for a single drone.

        This methods sequentially calls `_dslPIDPositionControl()` and `_dslPIDAttitudeControl()`.
        Parameter `cur_ang_vel` is unused.

        Parameters
        ----------
        control_timestep : float
            The time step at which control is computed.
        cur_pos : ndarray
            (3,1)-shaped array of floats containing the current position.
        cur_quat : ndarray
            (4,1)-shaped array of floats containing the current orientation as a quaternion.
        cur_vel : ndarray
            (3,1)-shaped array of floats containing the current velocity.
        cur_ang_vel : ndarray
            (3,1)-shaped array of floats containing the current angular velocity.
        target_pos : ndarray
            (3,1)-shaped array of floats containing the desired position.
        target_rpy : ndarray, optional
            (3,1)-shaped array of floats containing the desired orientation as roll, pitch, yaw.
        target_vel : ndarray, optional
            (3,1)-shaped array of floats containing the desired velocity.
        target_rpy_rates : ndarray, optional
            (3,1)-shaped array of floats containing the desired roll, pitch, and yaw rates.

        Returns
        -------
        ndarray
            (4,1)-shaped array of integers containing the RPMs to apply to each of the 4 motors.
        ndarray
            (3,1)-shaped array of floats containing the current XYZ position error.
        float
            The current yaw error.

        """
        self.control_counter += 1
        thrust, computed_target_rpy, pos_e = self._dslPIDVelocityControl(control_timestep,
                                                                         cur_quat,
                                                                         cur_vel,                                                                         
                                                                         target_rpy,
                                                                         target_vel
                                                                         )
        rpm = self._dslPIDAttitudeControl(control_timestep,
                                          thrust,
                                          cur_quat,
                                          computed_target_rpy,
                                          target_rpy_rates
                                          )
        cur_rpy = p.getEulerFromQuaternion(cur_quat)
        return rpm, pos_e, computed_target_rpy[2] - cur_rpy[2]
    
    ################################################################################

    def _dslPIDVelocityControl(self,
                               control_timestep,
                               cur_quat,
                               cur_vel,
                               target_rpy,
                               target_vel
                               ):
        """DSL's CF2.x PID position control.

        Parameters
        ----------
        control_timestep : float
            The time step at which control is computed.
        cur_pos : ndarray
            (3,1)-shaped array of floats containing the current position.
        cur_quat : ndarray
            (4,1)-shaped array of floats containing the current orientation as a quaternion.
        cur_vel : ndarray
            (3,1)-shaped array of floats containing the current velocity.
        target_pos : ndarray
            (3,1)-shaped array of floats containing the desired position.
        target_rpy : ndarray
            (3,1)-shaped array of floats containing the desired orientation as roll, pitch, yaw.
        target_vel : ndarray
            (3,1)-shaped array of floats containing the desired velocity.

        Returns
        -------
        float
            The target thrust along the drone z-axis.
        ndarray
            (3,1)-shaped array of floats containing the target roll, pitch, and yaw.
        float
            The current velocity error.

        """
        cur_rotation = np.array(p.getMatrixFromQuaternion(cur_quat)).reshape(3, 3)
        vel_e = target_vel - cur_vel
        
        # Termini integrali costruiti come finto integrale
        self.integral_vel_e = self.integral_vel_e + vel_e*control_timestep
        self.integral_vel_e = np.clip(self.integral_vel_e, -2., 2.)   # limita integral vel tra -2 e 2
        ## viene utilizzato per limitare gli errori integrali a un intervallo specifico per evitare l'integral windup, 
        ## che è un fenomeno dove l'errore integrale cresce troppo grande e causa instabilità nel sistema di controllo.
        self.integral_vel_e[2] = np.clip(self.integral_vel_e[2], -0.15, .15) # limita ulteriormente la velocità verticale PERCHè
        
        # Termini derivativi
        derivative_vel_e = (vel_e - self.prev_vel_e) / control_timestep

        # Aggiornare l'errore precedente per il prossimo passo
        self.prev_vel_e = vel_e

        #### PID target thrust #####################################
        target_thrust =   np.multiply(self.P_COEFF_VEL, vel_e) \
                        + np.multiply(self.I_COEFF_VEL, self.integral_vel_e) \
                        + np.multiply(self.D_COEFF_VEL, derivative_vel_e) \
                        + np.array([0, 0, self.GRAVITY])
        scalar_thrust = max(0., np.dot(target_thrust, cur_rotation[:,2]))
        thrust = (math.sqrt(scalar_thrust / (4*self.KF)) - self.PWM2RPM_CONST) / self.PWM2RPM_SCALE
        
        # calcolare l'orientamento desiderato (target rotation) del drone
        # basato sulla direzione della spinta calcolata (target thrust)
        target_z_ax = target_thrust / np.linalg.norm(target_thrust) #vettore unitario nella direz spinta (z)
        target_x_c = np.array([math.cos(target_rpy[2]), math.sin(target_rpy[2]), 0]) #vettore di riferimento per l'asse x 
        # target_x_c = np.array([1, 0, 0]) # Questo metodo fissa il vettore di riferimento per l'asse x nella direzione x globale, ignorando l'orientamento di yaw
        target_y_ax = np.cross(target_z_ax, target_x_c) / np.linalg.norm(np.cross(target_z_ax, target_x_c))
        target_x_ax = np.cross(target_y_ax, target_z_ax)
        target_rotation = (np.vstack([target_x_ax, target_y_ax, target_z_ax])).transpose() #matrice di rotazione
        
        #### Target rotation #######################################
        target_euler = (Rotation.from_matrix(target_rotation)).as_euler('XYZ', degrees=False)
        if np.any(np.abs(target_euler) > math.pi):
            print("\n[ERROR] ctrl it", self.control_counter, "in Control._dslPIDPositionControl(), values outside range [-pi,pi]")
        return thrust, target_euler, vel_e
    
    ################################################################################

    def _dslPIDAttitudeControl(self,
                                      control_timestep,
                                      thrust,
                                      cur_quat,
                                      target_euler,
                                      target_rpy_rates
                                      ):
        """DSL's CF2.x PID attitude control.

        Parameters
        ----------
        control_timestep : float
            The time step at which control is computed.
        thrust : float
            The target thrust along the drone z-axis.
        cur_quat : ndarray
            (4,1)-shaped array of floats containing the current orientation as a quaternion.
        target_euler : ndarray
            (3,1)-shaped array of floats containing the computed target Euler angles.
        target_rpy_rates : ndarray
            (3,1)-shaped array of floats containing the desired roll, pitch, and yaw rates.

        Returns
        -------
        ndarray
            (4,1)-shaped array of integers containing the RPMs to apply to each of the 4 motors.

        """
        cur_rotation = np.array(p.getMatrixFromQuaternion(cur_quat)).reshape(3, 3)
        cur_rpy = np.array(p.getEulerFromQuaternion(cur_quat))
        target_quat = (Rotation.from_euler('XYZ', target_euler, degrees=False)).as_quat()
        w,x,y,z = target_quat
        target_rotation = (Rotation.from_quat([w, x, y, z])).as_matrix()
        rot_matrix_e = np.dot((target_rotation.transpose()),cur_rotation) - np.dot(cur_rotation.transpose(),target_rotation)
        rot_e = np.array([rot_matrix_e[2, 1], rot_matrix_e[0, 2], rot_matrix_e[1, 0]]) 
        rpy_rates_e = target_rpy_rates - (cur_rpy - self.last_rpy)/control_timestep
        self.last_rpy = cur_rpy
        self.integral_rpy_e = self.integral_rpy_e - rot_e*control_timestep
        self.integral_rpy_e = np.clip(self.integral_rpy_e, -1500., 1500.)
        self.integral_rpy_e[0:2] = np.clip(self.integral_rpy_e[0:2], -1., 1.)
        #### PID target torques ####################################
        target_torques = - np.multiply(self.P_COEFF_TOR, rot_e) \
                         + np.multiply(self.D_COEFF_TOR, rpy_rates_e) \
                         + np.multiply(self.I_COEFF_TOR, self.integral_rpy_e)
        target_torques = np.clip(target_torques, -3200, 3200)
        pwm = thrust + np.dot(self.MIXER_MATRIX, target_torques)
        pwm = np.clip(pwm, self.MIN_PWM, self.MAX_PWM)
        return self.PWM2RPM_SCALE * pwm + self.PWM2RPM_CONST
    
    ################################################################################

    def _dslPIDAngularVelocityControl(self,
                                       control_timestep,
                                       cur_ang_vel,
                                       target_ang_vel
                                  ):
        """DSL's CF2.x PID angular velocity control.

        Parameters
        ----------
        control_timestep : float
            The time step at which control is computed.
        cur_ang_vel : ndarray
            (3,1)-shaped array of floats containing the current angular velocity.
        target_ang_vel : ndarray
            (3,1)-shaped array of floats containing the desired angular velocity.

        Returns
        -------
        ndarray
            (4,1)-shaped array of integers containing the RPMs to apply to each of the 4 motors.

        """
        # Compute the angular velocity error
        ang_vel_e = target_ang_vel - cur_ang_vel

        # Update the integral of the angular velocity error
        self.integral_rpy_e += ang_vel_e * control_timestep
        self.integral_rpy_e = np.clip(self.integral_rpy_e, -1500., 1500.)
        self.integral_rpy_e[0:2] = np.clip(self.integral_rpy_e[0:2], -1., 1.)

        # Compute the derivative of the angular velocity error
        derivative_rpy_e = (ang_vel_e - self.last_rpy_e) / control_timestep
        self.last_rpy_e = ang_vel_e

        # Compute the PID target torques
        target_torques = np.multiply(self.P_COEFF_TOR, ang_vel_e) \
                         + np.multiply(self.I_COEFF_TOR, self.integral_rpy_e) \
                         + np.multiply(self.D_COEFF_TOR, derivative_rpy_e)

        # Clip the target torques to the maximum allowable values
        target_torques = np.clip(target_torques, -3200, 3200)

        # Compute the PWM signal for each motor
        pwm = thrust + np.dot(self.MIXER_MATRIX, target_torques)
        pwm = np.clip(pwm, self.MIN_PWM, self.MAX_PWM)

        # Convert the PWM signal to RPM
        return self.PWM2RPM_SCALE * pwm + self.PWM2RPM_CONST

    def _one23DInterface(self,
                         thrust
                         ):
        """Utility function interfacing 1, 2, or 3D thrust input use cases.

        Parameters
        ----------
        thrust : ndarray
            Array of floats of length 1, 2, or 4 containing a desired thrust input.

        Returns
        -------
        ndarray
            (4,1)-shaped array of integers containing the PWM (not RPMs) to apply to each of the 4 motors.

        """
        DIM = len(np.array(thrust))
        pwm = np.clip((np.sqrt(np.array(thrust)/(self.KF*(4/DIM)))-self.PWM2RPM_CONST)/self.PWM2RPM_SCALE, self.MIN_PWM, self.MAX_PWM)
        if DIM in [1, 4]:
            return np.repeat(pwm, 4/DIM)
        elif DIM==2:
            return np.hstack([pwm, np.flip(pwm)])
        else:
            print("[ERROR] in DSLPIDControl._one23DInterface()")
            exit()
