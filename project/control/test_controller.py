# questo test prova a utilizzare il controllore di velocità
import numpy as np
import math
import pybullet as p
from scipy.spatial.transform import Rotation

class DroneVelocityController:
    def __init__(self, P_COEFF_VEL, I_COEFF_VEL, D_COEFF_VEL, GRAVITY, KF, PWM2RPM_CONST, PWM2RPM_SCALE):
        self.P_COEFF_VEL = P_COEFF_VEL
        self.I_COEFF_VEL = I_COEFF_VEL
        self.D_COEFF_VEL = D_COEFF_VEL
        self.GRAVITY = GRAVITY
        self.KF = KF
        self.PWM2RPM_CONST = PWM2RPM_CONST
        self.PWM2RPM_SCALE = PWM2RPM_SCALE
        self.integral_vel_e = np.zeros(3)
        self.prev_vel_e = np.zeros(3)  # Per memorizzare l'errore precedente

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
    
# Esempio di utilizzo
P_COEFF_VEL = np.array([1.0, 1.0, 1.0])     # requires tuning
I_COEFF_VEL = np.array([0.1, 0.1, 0.1])     # requires tuning
D_COEFF_VEL = np.array([0.01, 0.01, 0.01])  # requires tuning
GRAVITY = 9.81
KF = 0.000015
PWM2RPM_CONST = 4000
PWM2RPM_SCALE = 0.2685

controller = DroneVelocityController(P_COEFF_VEL, I_COEFF_VEL, D_COEFF_VEL, GRAVITY, KF, PWM2RPM_CONST, PWM2RPM_SCALE)

duration_sec = 10
control_timestep = 0.1
control_freq = 1/control_timestep
cur_pos = np.array([0.0, 0.0, 1.0])
cur_quat = np.array([0.0, 0.0, 0.0, 1.0])
cur_vel = np.array([0.0, 0.0, 0.0])
target_rpy = np.array([0.0, 0.0, 0.0])
target_vel = np.array([1.0, 0.0, 0.0])

for i in range(0, int(duration_sec*control_freq)):
    thrust, target_euler, vel_e = controller._dslPIDVelocityControl(control_timestep, cur_quat, cur_vel, target_rpy, target_vel)
    print("Thrust:", thrust)
    print("Target Euler angles:", target_euler)
    print("Velocity error:", vel_e)    