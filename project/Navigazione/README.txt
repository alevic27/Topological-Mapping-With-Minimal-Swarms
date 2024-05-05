in BaseAviary ho aggiunto 
  def __init__(self,
                 drone_model: DroneModel=DroneModel.CF2X,
                 num_drones: int=1,
                 neighbourhood_radius: float=np.inf,
                 initial_xyzs=None,
                 initial_rpys=None,
                 physics: Physics=Physics.PYB,
                 pyb_freq: int = 240,
                 ctrl_freq: int = 240,
                 gui=False,
                 record=False,
                 obstacles=False,
                 user_debug_gui=True,
                 vision_attributes=False,
                 sensors_attributes=True,  <----- deve stare su True per prendere i sensori
                 output_folder='results'
                 ):

da riga 130 
### Create attributes for sensors ####################
# Aggiungo tot sensori al drone
        self.SENSOR_ATTR = sensors_attributes 
        if self.SENSOR_ATTR :
            self.NUM_SENSORS = 4 
            self.sensor_position =np.zeros((4,1))  # Posizione del sensore (x, y, z)
            self.sensor_direction =[[1, 0 , 0],
                                    [-1, 0 , 0],
                                    [0, 1, 0],
                                    [0, -1, 0]]  # Direzione del sensore (x, y, z)
            self.max_range = 2  # Lunghezza massima del sensore

in CtrlAviary ho aggiunto da riga 121
def _sensorsObs(self):
        observation = np.array([[1000000, 1000000, 1000000] for i in range(self.NUM_DRONES)])
        if self.SENSOR_ATTR:
            for i in range(self.NUM_DRONES):
                closest_hit = []
                for j in range(self.NUM_SENSORS) :
                    self.sensor_position = self._getDroneStateVector(i)[0:3]
                    result =p.rayTest(self.sensor_position, 
                                        [self.sensor_position[0] + self.max_range * self.sensor_direction[j][0], 
                                        self.sensor_position[1] + self.max_range * self.sensor_direction[j][1], 
                                        self.sensor_position[2] + self.max_range * self.sensor_direction[j][2]])
                    closest_hit.append(result[0])
                   #closest_hit[j][0] = result_list[j][0]
                    # Se c'è un punto di contatto, ottieni la distanza
                    if closest_hit[j][0] != -1:
                        hit_distance = closest_hit[j][2]
                        hit_position = closest_hit[j][3]
                        # Aggiungi la distanza rilevata come osservazione
                        observation[i][j]= hit_distance
                    else:
                        # Se non ci sono oggetti rilevati, assegna una distanza massima
                        observation[i][j] =  self.max_range
        return observation

in prova1.py ho modificato la generazione di traiettoria e aggiunto le letture del sensore
teoricamente lanciando questo con le modifiche precedenti dovrebbe funzionare
