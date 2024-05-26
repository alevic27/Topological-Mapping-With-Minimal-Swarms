import os
import time
from datetime import datetime
import argparse
import gymnasium as gym
import numpy as np
import csv
# import torch
# from stable_baselines3 import PPO
# from stable_baselines3.common.env_util import make_vec_env
# from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold
# from stable_baselines3.common.evaluation import evaluate_policy

from gym_pybullet_drones.utils.Logger import Logger
from project.envs.TopoAviary import TopoAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
# from gym_pybullet_drones.envs.MultiHoverAviary import MultiHoverAviary
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
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_PLOT = True
DEFAULT_SIMULATION_FREQ_HZ = 100
DEFAULT_CONTROL_FREQ_HZ = 100
DEFAULT_DURATION_SEC = 10
DEFAULT_NUM_DRONES = 1
DEFAULT_IMG_RES = np.array([64, 48])

DEFAULT_LABYRINTH_ID = "2t"  # "0" per i 4 oggettini di BaseRLAviary, "lettera" della versione del labirinto 

def run(
        output_folder=DEFAULT_OUTPUT_FOLDER,
        colab=DEFAULT_COLAB, 
        record_video=DEFAULT_SAVE_IMAGES, 
        local=True,
        drone=DEFAULT_DRONES,
        num_drones=DEFAULT_NUM_DRONES,
        physics=DEFAULT_PHYSICS,
        gui=DEFAULT_GUI,
        plot=DEFAULT_PLOT,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        duration_sec=DEFAULT_DURATION_SEC,
        labyrinth_id=DEFAULT_LABYRINTH_ID,
        ):
    
    INIT_XYZ = np.array([[3, 0, 1] for i in range(1,num_drones+1)])
    INIT_RPY = np.array([[.0, .0, .0] for _ in range(num_drones)])
    env = TopoAviary(drone_model=drone,
                     num_drones=num_drones,
                     initial_xyzs=INIT_XYZ,
                     initial_rpys=INIT_RPY,
                     physics=physics,
                     pyb_freq=simulation_freq_hz,
                     ctrl_freq=control_freq_hz,
                     gui=gui,   
                     record=DEFAULT_SAVE_IMAGES,
                     labyrinth_id=labyrinth_id,
                     obs=DEFAULT_OBS,    
                     img_res=DEFAULT_IMG_RES                
                     )
    
    #### Initialize the controllers ############################
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

        #### Make it rain rubber ducks #############################
        # if i/env.SIM_FREQ>5 and i%10==0 and i/env.SIM_FREQ<10: p.loadURDF("duck_vhacd.urdf", [0+random.gauss(0, 0.3),-0.5+random.gauss(0, 0.3),3], p.getQuaternionFromEuler([random.randint(0,360),random.randint(0,360),random.randint(0,360)]), physicsClientId=PYB_CLIENT)

        #### Step the simulation ###################################
        obs, reward, terminated, truncated, info = env.step(action)

        #### Compute control for the current way point #############
        
        for j in range(num_drones):
            action[j, :], _, _ = ctrl[j].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                    state=obs[j],
                                                                    target_pos=INIT_XYZ[j, :],
                                                                    target_rpy=INIT_RPY[j, :]
                                                                    )
        '''
        print('PRINTANDO obs')
        print(obs)
        print('PRINTANDO obs.shape')
        print(obs.shape)
        print('PRINTANDO obs[0]')
        state = obs[0]
        print(state)
        '''

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
    parser.add_argument('--record_video',       default=DEFAULT_SAVE_IMAGES,      type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--plot',               default=DEFAULT_PLOT,       type=str2bool,      help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ,        type=int,           help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz',    default=DEFAULT_CONTROL_FREQ_HZ,         type=int,           help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec',       default=DEFAULT_DURATION_SEC,         type=int,           help='Duration of the simulation in seconds (default: 5)', metavar='')
    parser.add_argument('--output_folder',     default=DEFAULT_OUTPUT_FOLDER, type=str,           help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--colab',              default=DEFAULT_COLAB, type=bool,           help='Whether example is being run by a notebook (default: "False")', metavar='')
    ARGS = parser.parse_args()

    run(**vars(ARGS))

    
