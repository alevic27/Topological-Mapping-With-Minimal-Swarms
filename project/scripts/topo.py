import os
import time
from datetime import datetime
import argparse
import gymnasium as gym
import numpy as np
import csv

from gym_pybullet_drones.utils.Logger import Logger
from project.envs.MapAviary import MapAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.utils import sync, str2bool
from gym_pybullet_drones.utils.enums import DroneModel, Physics, ActionType, ObservationType

DEFAULT_GUI = True
DEFAULT_SAVE_IMAGES = True # True per salvare foto
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = True

DEFAULT_OBS = ObservationType('rgb') # 'kin' or 'rgb'
DEFAULT_ACT = ActionType('one_d_rpm') # 'rpm' or 'pid' or 'vel' or 'one_d_rpm' or 'one_d_pid'
DEFAULT_AGENTS = 2

DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_NUM_DRONES = 1
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_PLOT = True
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_DURATION_SEC = 240

DEFAULT_IMG_RES = np.array([64, 48])

DEFAULT_SENSORS_ATTRIBUTES = True
DEFAULT_SENSORS_RANGE = 4.
DEFAULT_REF_DISTANCE = 0.3
DEFAULT_LABYRINTH_ID = "2t"  # "0" per i 4 oggettini di BaseRLAviary, "lettera" della versione del labirinto 
DEFAULT_S_WF: int = +1   #wallfollowing side
DEFAULT_CONTROL_OMEGA : float = 0.5  #works with 0.5
DEFAULT_CONTROL_VELOCITY: float = 0.2
DEFAULT_WFSTATE : int = -1
DEFAULT_THRESHOLD_DISTANCE : float = 0.04

def run(
        drone=DEFAULT_DRONES,
        num_drones=DEFAULT_NUM_DRONES,
        physics=DEFAULT_PHYSICS,
        gui=DEFAULT_GUI,
        plot=DEFAULT_PLOT,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        duration_sec=DEFAULT_DURATION_SEC,
        sensors_attributes= DEFAULT_SENSORS_ATTRIBUTES,
        max_sensors_range = DEFAULT_SENSORS_RANGE,
        ref_distance = DEFAULT_REF_DISTANCE,
        labyrinth_id=DEFAULT_LABYRINTH_ID,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        colab=DEFAULT_COLAB,
        s_WF=DEFAULT_S_WF,
        c_omega=DEFAULT_CONTROL_OMEGA,
        c_vel=DEFAULT_CONTROL_VELOCITY,
        WFstate=DEFAULT_WFSTATE, 
        td=DEFAULT_THRESHOLD_DISTANCE
        ):
    
    ### definisci le posizioni iniziali dei droni
    H_ini = 1.
    X_ini = 0.
    Y_ini = 0.
    Y_STEP_ini = .05
    #posizione e attitude iniziale per ogni drone 
    INIT_XYZS = np.array([[X_ini,Y_ini+i*Y_STEP_ini, H_ini] for i in range(num_drones)])
    INIT_RPYS = np.array([[0., 0., 0.] for i in range(num_drones)])
    
    ### WP target

    timer = 0
    #for i in range(NUM_WP):
    #    TARGET_POS[i,:]= X_ini + i*(X_target-X_ini)/NUM_WP,Y_ini + i* (Y_target-Y_ini)/NUM_WP, H_target
    wp_counters = np.array([0 for i in range(num_drones)])  #comodo quando si fisseranno i primi nodi
    
    env = MapAviary(drone_model=drone,
                    num_drones=num_drones,
                    neighbourhood_radius=10,
                    initial_xyzs=INIT_XYZS,
                    initial_rpys=INIT_RPYS,
                    physics=physics,
                    pyb_freq=simulation_freq_hz,
                    ctrl_freq=control_freq_hz,
                    gui=gui,   
                    vision = False,
                    img_res  = np.array([64, 48]),
                    save_imgs=False,
                    obstacle_ids=False,
                    labyrinth_id = labyrinth_id,
                    #record=DEFAULT_SAVE_IMAGES,
                    sensors_attributes = sensors_attributes,
                    max_sensors_range = max_sensors_range,
                    ref_distance = ref_distance,
                    output_folder='output_folder',
                    s_WF=s_WF,
                    c_omega=c_omega,
                    c_vel=c_vel,
                    WFstate=WFstate,
                    td=td               
                    )
    
    #### Initialize the controllers ############################
    if drone in [DroneModel.CF2X, DroneModel.CF2P]:
        ctrl = [DSLPIDControl(drone_model=drone) for i in range(num_drones)]

    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()
    DRONE_IDS = env.getDroneIds()
    
    #### Initialize the logger #################################
    logger = Logger(logging_freq_hz=control_freq_hz,
                    num_drones=num_drones,
                    output_folder=output_folder,
                    colab=colab
                    )
    
    #### Run the simulation ####################################
    action = np.zeros((num_drones,4))
    START = time.time()
    for i in range(0, int(duration_sec*env.CTRL_FREQ)):

        #### Step the simulation ###################################
        obs, observation, reward, terminated, truncated, info = env.step(action)

        #### Compute control for the current way point #############
        TARGET_POS , TARGET_RPY , TARGET_VEL , TARGET_RPY_RATES = env.NextWP(obs,observation)       

        # velocity control PLACEHOLDER #TO BE IMPLEMENTED
        #TARGET_VEL , TARGET_RPY_RATES = env.NextWP_VEL(obs,observation) #

        # applica i controlli in modo da raggiungere un certo punto con un certa velocità
        for j in range(num_drones) :
            action[j, :], _, _ = ctrl[j].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                    state=obs[j],
                                                                    target_pos=TARGET_POS[j],
                                                                    target_rpy=TARGET_RPY[j],
                                                                    target_vel = TARGET_VEL[j],
                                                                    target_rpy_rates = TARGET_RPY_RATES[j]
                                                                    )
       
        #### Log the simulation ####################################
        for j in range(num_drones):
            logger.log(drone=j,
                       timestamp=i/env.CTRL_FREQ,
                       state=obs[j],
                       # control=np.hstack([TARGET_POS[wp_counters[j], 0:2], INIT_XYZS[j, 2], INIT_RPYS[j, :], np.zeros(6)])
                       # control=np.hstack([INIT_XYZS[j, :]+TARGET_POS[wp_counters[j], :], INIT_RPYS[j, :], np.zeros(6)])
                       )
        

      

        #### Printout ##############################################
        env.render()

        #### Sync the simulation ###################################
        if gui:
            sync(i, START, env.CTRL_TIMESTEP)

    #### Close the environment #################################
    env.close()

    #### Save the simulation results ###########################
    logger.save()
    logger.save_as_csv("camera_tests") # Optional CSV save

    #### Plot the simulation results ###########################
    if plot:
        logger.plot()
    
if __name__ == "__main__":
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Helix flight script using CtrlAviary and DSLPIDControl')
    parser.add_argument('--drone',              default=DEFAULT_DRONES,     type=DroneModel,    help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
    parser.add_argument('--num_drones',         default=DEFAULT_NUM_DRONES,          type=int,           help='Number of drones (default: 3)', metavar='')
    parser.add_argument('--physics',            default=DEFAULT_PHYSICS,      type=Physics,       help='Physics updates (default: PYB)', metavar='', choices=Physics)
    parser.add_argument('--gui',                default=DEFAULT_GUI,       type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
#    parser.add_argument('--record_video',       default=DEFAULT_SAVE_IMAGES,      type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--plot',               default=DEFAULT_PLOT,       type=str2bool,      help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ,        type=int,           help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz',    default=DEFAULT_CONTROL_FREQ_HZ,         type=int,           help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec',       default=DEFAULT_DURATION_SEC,         type=int,           help='Duration of the simulation in seconds (default: 5)', metavar='')
    parser.add_argument('--output_folder',      default=DEFAULT_OUTPUT_FOLDER, type=str,           help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--colab',              default=DEFAULT_COLAB, type=bool,           help='Whether example is being run by a notebook (default: "False")', metavar='')
    ARGS = parser.parse_args()

    run(**vars(ARGS))
