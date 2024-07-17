from enum import Enum as assets_list
from shapely.geometry import Polygon
import numpy as np

class Labyrinth(assets_list):

    BASEAVIARY = "0"
    DOUBLE_T = "2t"   
    DOUBLE_T_2X = "2t_2x"   # scalato 2x
    DOUBLE_T_V2 = "2t_v2"   # scalato 2x tenendo i corridoi della stessa larghezza

# Definizione del dizionario con i dati dei labirinti
LABYRINTH_CONFIG = {
    Labyrinth.DOUBLE_T: {
        "name": "Labirinto Doppia T",
        "polygon": Polygon([(3.0, 1.0), (1.0, 1.0), (1.0, 3.1), (2.9, 3.1), (2.9 , 4.9),(-2.9 , 4.9),(-2.9 , 3.1),
                                (-1, 3.1), (-1, -3.1),(-2.9, -3.1),(-2.9, -4.9),(2.9, -4.9),
                                (2.9 , -3.1),(1., -3.1),(1., -1.0),(3., -1.)]),
        "starting_coords_offset": np.array([-0.5, 0.0, 0.0])
    },
    Labyrinth.DOUBLE_T_2X: {
        "name": "Labirinto Doppia T scalato 2x in direzioni X e Y",
        "polygon": Polygon([(6.0, 2.0), (2.0, 2.0), (2.0, 6.2), (5.8, 6.2), (5.8, 9.8), (-5.8, 9.8), (-5.8, 6.2),
                                (-2.0, 6.2), (-2.0, -6.2), (-5.8, -6.2), (-5.8, -9.8), (5.8, -9.8), (5.8, -6.2),
                                (2.0, -6.2), (2.0, -2.0), (6.0, -2.0)]),
        "starting_coords_offset": np.array([-2.0, 0.0, 0.0])
    },
    Labyrinth.DOUBLE_T_V2: {
        "name": "Labirinto Doppia T scalato 2x in direzioni X e Y v2",
        "polygon": Polygon([(6.0, 1.0), (1.0, 1.0), (1.0, 8.0), (5.8, 8.0), (5.8, 9.8), (-5.8, 9.8), (-5.8, 8.0),
                                (-1.0, 8.0), (-1.0, -8.0), (-5.8, -8.0), (-5.8, -9.8), (5.8, -9.8), (5.8, -8.0),
                                (1.0, -8.0), (1.0, -1.0), (6.0, -1.0)]),
        "starting_coords_offset": np.array([-3.0, 0.0, 0.0])
    }
    # Aggiungi altri labirinti qui con le loro configurazioni
}







    