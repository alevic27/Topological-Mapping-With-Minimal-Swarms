import os
import time
import argparse
from datetime import datetime
import pdb
import math
import random
import numpy as np
import pybullet as p
import gymnasium as gym #per i problemi di apprendimento mi serve la libreira gymnasium
import matplotlib.pyplot as plt

from gym_pybullet_drones.utils.enums import DroneModel, Physics
# importa una una classe che descrive il controllo del drone
from project.envs.MapAviary import MapAviary
# importa il file .py con la definizione di tutte le function necessarie al controllo PID 
#(ci v messo per forza un drone CF2X o CF"P perchè presenta parametri
#di controllo solo per questi due tipi)

# importa un file .py in cui viene definita una classe che definisce un ambiente adatto ai problemi di apprendimento multi and single angent 
'''from gym_pybullet_drones.envs.HoverAviary import HoverAviary
from gym_pybullet_drones.envs.MultiHoverAviary import MultiHoverAviary'''

from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl 
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool
#importa due classi , la prima definisce il modo in cui avvengono le osservazioni del drone, 
#from gym_pybullet_drones.utils.enums import ObservationType, ActionType


DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_NUM_DRONES = 1
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_RECORD_VISION = False
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_OBSTACLES = True 
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_DURATION_SEC = 50
DEFAULT_SENSORS_ATTRIBUTES = True
DEFAULT_SENSORS_RANGE = 4.,
DEFAULT_REF_DISTANCE = 1.,
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False

# 
''' DEFAULT_OBS = ObservationType.KIN # 'kin' or 'rgb'
DEFAULT_ACT = ActionType.RPM # 'rpm' or 'pid' or 'vel' or 'one_d_rpm' or 'one_d_pid'
DEFAULT_AGENTS = 2
DEFAULT_MA = False '''

def run(
        drone=DEFAULT_DRONES,
        num_drones=DEFAULT_NUM_DRONES,
        physics=DEFAULT_PHYSICS,
        gui=DEFAULT_GUI,
        record_video=DEFAULT_RECORD_VISION,
        plot=DEFAULT_PLOT,
        user_debug_gui=DEFAULT_USER_DEBUG_GUI,
        obstacles=DEFAULT_OBSTACLES,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        duration_sec=DEFAULT_DURATION_SEC,
        sensors_attributes= DEFAULT_SENSORS_ATTRIBUTES,
        max_sensors_range = DEFAULT_SENSORS_RANGE,
        ref_distance = DEFAULT_REF_DISTANCE,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        colab=DEFAULT_COLAB,
        #obs_type=DEFAULT_OBS,
        #act_type=DEFAULT_ACT 
        ):
    #### Initialize the simulation #############################
    H_ini = .1
    X_ini = 0
    Y_ini = 0
    Y_STEP_ini = .05 # in caso di multi agenti li faccio partire in posizioni diverse (possibilmente davanti all'ingresso)
    #posizione e velocità iniziale per ogni drone 
    INIT_XYZS = np.array([[X_ini,Y_ini+i*Y_STEP_ini, H_ini] for i in range(num_drones)])
    INIT_RPYS = np.array([[0., 0., 0.] for i in range(num_drones)])

    #### WP target ######################
    #inserisco per ota un unico WP da raggiungere
    TARGET_POS = INIT_XYZS
    TARGET_RPY =  INIT_RPYS
   
    timer = 0
    #for i in range(NUM_WP):
    #    TARGET_POS[i,:]= X_ini + i*(X_target-X_ini)/NUM_WP,Y_ini + i* (Y_target-Y_ini)/NUM_WP, H_target
    wp_counters = np.array([0 for i in range(num_drones)])  #comodo quando si fisseranno i primi nodi
    
    #### Create the environment ################################
    env = MapAviary (  drone_model = drone ,
                        num_drones = DEFAULT_NUM_DRONES,
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
                        labyrinth_id = "0",
                        sensors_attributes = sensors_attributes,
                        max_sensors_range = max_sensors_range,
                        ref_distance = ref_distance,
                        output_folder='results'
                        )
    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()

    #### Initialize the logger #################################
    logger = Logger(logging_freq_hz=control_freq_hz,
                    num_drones=num_drones,
                    output_folder=output_folder,
                    colab=colab
                    )

    #### Initialize the controllers ############################
    if drone in [DroneModel.CF2X, DroneModel.CF2P]:
        ctrl = [DSLPIDControl(drone_model=drone) for i in range(num_drones)]

    #### Run the simulation ####################################
    action = np.zeros((num_drones,4))
    START = time.time()
    for i in range(0, int(duration_sec*env.CTRL_FREQ)):

        #### Step the simulation ###################################
        obs, observation, reward, terminated, truncated, info = env.step(action)

        #### Compute control for the current way point #############
        TARGET_POS , TARGET_RPY ,TARGET_VEL , TARGET_OMEGA = env.NextWP(obs,observation)
           
        # applica i controlli in modo da raggiungere un certo punto con un certa velocità
        for j in range(num_drones) :
            action[j, :], _, _ = ctrl[j].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                    state=obs[j],
                                                                    target_pos=TARGET_POS[j],
                                                                    target_rpy=TARGET_RPY[j],
                                                                    target_vel = TARGET_VEL[j],
                                                                    target_rpy_rates = TARGET_OMEGA[j]
                                                                    )

                    
        


        #### Log the simulation ####################################
        for j in range(num_drones):
            logger.log(drone=j,
                       timestamp=i/env.CTRL_FREQ,
                       state=obs[j],
                       #control=np.hstack([TARGET_POS[wp_counters[j]][0:2], INIT_XYZS[j, 2], INIT_RPYS[j, :], np.zeros(6)])
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
    logger.save_as_csv("pid") # Optional CSV save

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
    parser.add_argument('--record_video',       default=DEFAULT_RECORD_VISION,      type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--plot',               default=DEFAULT_PLOT,       type=str2bool,      help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--user_debug_gui',     default=DEFAULT_USER_DEBUG_GUI,      type=str2bool,      help='Whether to add debug lines and parameters to the GUI (default: False)', metavar='')
    parser.add_argument('--obstacles',          default=DEFAULT_OBSTACLES,       type=str2bool,      help='Whether to add obstacles to the environment (default: True)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ,        type=int,           help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz',    default=DEFAULT_CONTROL_FREQ_HZ,         type=int,           help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec',       default=DEFAULT_DURATION_SEC,         type=int,           help='Duration of the simulation in seconds (default: 5)', metavar='')
    parser.add_argument('--output_folder',     default=DEFAULT_OUTPUT_FOLDER, type=str,           help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--colab',              default=DEFAULT_COLAB, type=bool,           help='Whether example is being run by a notebook (default: "False")', metavar='')
    ARGS = parser.parse_args()

    run(**vars(ARGS))

