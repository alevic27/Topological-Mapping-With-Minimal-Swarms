"""
This code tests the _storeHitPoints(self, Hit_point) method of the ProjAviary class
"""

import numpy as np
from project.envs.ProjAviary import ProjAviary

Hit_point = np.array([
    [
        [1, 2, 3],
        [4, 5, 6],
        [np.inf, 8, 9],
        [10, 11, 12]
    ],
    [
        [1, 2, 3],
        [4, 5, 6],
        [7, 8, 9],
        [np.inf, np.inf, np.inf]
    ],
    [
        [1, 2, 3],
        [4, 5, 6],
        [7, 8, 9],
        [10, 11, 12]
    ]
])

env = ProjAviary(num_drones=3, gui=False)
env._storeHitPoints(Hit_point)

print(env.CLOUD_POINT)

