import time
import argparse
import numpy as np

from gym_pybullet_drones.utils.utils import sync, str2bool
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger



DEFAULT_DRONE = DroneModel('cf2x')
DEFAULT_NUM_DRONES = 1
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_RECORD_VISION = False
DEFAULT_PLOT = False
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_OBSTACLES = False
DEFAULT_RECORD_VIDEO = False
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_DURATION_SEC = 30
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False


def run(
        drone=DEFAULT_DRONE, 
        num_drones=DEFAULT_NUM_DRONES,
        gui=DEFAULT_GUI, 
        obstacles=DEFAULT_OBSTACLES,
        record_video=DEFAULT_RECORD_VIDEO, 
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ, 
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ, 
        duration_sec=DEFAULT_DURATION_SEC,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        plot=DEFAULT_PLOT,
        colab=DEFAULT_COLAB
    ):

    #### Initialize the simulation #############################

    INIT_XYZS = np.array([[ 0, 0, .1]])
    # per inizializzare più droni
    #INIT_XYZS = np.array([[R*np.cos((i/6)*2*np.pi+np.pi/2), R*np.sin((i/6)*2*np.pi+np.pi/2)-R, H+i*H_STEP] for i in range(num_drones)])
    # per inizializzare l'orientazione
    #INIT_RPYS = np.array([[0, 0,  i * (np.pi/2)/num_drones] for i in range(num_drones)])

    env = CtrlAviary(drone_model=drone,
                     num_drones=DEFAULT_NUM_DRONES,
                     initial_xyzs=INIT_XYZS,
                     #initial_rpys=INIT_RPYS,
                     physics=Physics.PYB_DW,
                     neighbourhood_radius=10,
                     pyb_freq=simulation_freq_hz,
                     ctrl_freq=control_freq_hz,
                     gui=gui,
                     record=record_video,
                     obstacles=obstacles
                     )
    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()
    DRONE_IDS = env.getDroneIds()

    #### Initialize the trajectories ###########################
    PERIOD = 5
    NUM_WP = control_freq_hz*PERIOD
    TARGET_POS = np.zeros((NUM_WP, 2))
    for i in range(NUM_WP):
        TARGET_POS[i, :] = [0.5*np.cos(2*np.pi*(i/NUM_WP)), 0]
    wp_counters = np.array([0, int(NUM_WP/2)])

    #### Initialize the logger #################################
    logger = Logger(logging_freq_hz=control_freq_hz,
                    num_drones=num_drones,
                    output_folder=output_folder,
                    colab=colab
                    )
    
    #### Initialize the controllers ############################
    ctrl = [DSLPIDControl(drone_model=drone) for i in range(2)]

    #### Run the simulation ####################################
    action = np.zeros((num_drones,4))
    START = time.time()
    for i in range(0, int(duration_sec*env.CTRL_FREQ)):
        
        #### Step the simulation ###################################
        obs, reward, terminated, truncated, info = env.step(action)

        #### Compute control for the current way point #############
        for j in range(num_drones):
            action[j, :], _, _ = ctrl[j].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                    state=obs[j],
                                                                    target_pos=np.hstack([TARGET_POS[wp_counters[j], :], INIT_XYZS[j, 2]]),
                                                                    #target_rpy=INIT_RPYS[j, :]
                                                                   )
        #### Go to the next way point and loop #####################
        for j in range(num_drones):
            wp_counters[j] = wp_counters[j] + 1 if wp_counters[j] < (NUM_WP-1) else 0

        #### Log the simulation ####################################
        for j in range(num_drones):
            logger.log(drone=j,
                       timestamp=i/env.CTRL_FREQ,
                       state=obs[j],
                       control=np.hstack([TARGET_POS[wp_counters[j], :], INIT_XYZS[j, 2], np.zeros(9)])
                       #control=np.hstack([TARGET_POS[wp_counters[j], :], INIT_XYZS[j, 2], INIT_RPYS[j, :], np.zeros(6)])
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
    logger.save_as_csv("flight_test_1") # Optional CSV save
    if plot:
        logger.plot()
if __name__ == "__main__":
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Downwash example script using CtrlAviary and DSLPIDControl')
    parser.add_argument('--drone',              default=DEFAULT_DRONE,     type=DroneModel,    help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
    parser.add_argument('--gui',                default=DEFAULT_GUI,       type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video',       default=DEFAULT_RECORD_VIDEO,      type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ,        type=int,           help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz',    default=DEFAULT_CONTROL_FREQ_HZ,         type=int,           help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec',       default=DEFAULT_DURATION_SEC,         type=int,           help='Duration of the simulation in seconds (default: 10)', metavar='')
    parser.add_argument('--output_folder',     default=DEFAULT_OUTPUT_FOLDER, type=str,           help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--colab',              default=DEFAULT_COLAB, type=bool,           help='Whether example is being run by a notebook (default: "False")', metavar='')
    ARGS = parser.parse_args()

    run(**vars(ARGS))