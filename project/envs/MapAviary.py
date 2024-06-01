import numpy as np
import pybullet as p
from gymnasium import spaces
import pkg_resources
from project.envs.ProjAviary import ProjAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics, ImageType

class MapAviary(ProjAviary):
    "Multi-drone environment class for topological mapping of unknown places"

    ################################################################################

    def __init__(self,
                 drone_model: DroneModel=DroneModel.CF2X,
                 num_drones: int=1,
                 neighbourhood_radius: float=np.inf,
                 initial_xyzs=None,
                 initial_rpys=None,
                 pyb_freq: int = 240,
                 ctrl_freq: int = 240,
                 gui=True,
                 vision=False,
                 img_res: np.ndarray=np.array([64, 48]),
                 save_imgs=False,
                 obstacle_ids=False,
                 labyrinth_id: str = "0",
                 sensors_attributes = True,
                 max_sensors_range: float = np.inf,
                 ref_distance : float = 0.5,
                 s_WF: int = -1,
                 c_omega : float = 0.,
                 c_vel: float = 0.,
                 output_folder='results',
                 WFstate : int = 0,
                 ):
        
        """Initialization of an environment with drones capable of performing topological mapping.

        Parameters
        ----------
        drone_model : DroneModel, optional
            The desired drone type (detailed in an .urdf file in folder `assets`).
        num_drones : int, optional
            The desired number of drones in the aviary.
        neighbourhood_radius : float, optional
            Radius used to compute the drones' adjacency matrix, in meters.
        initial_xyzs: ndarray | None, optional
            (NUM_DRONES, 3)-shaped array containing the initial XYZ position of the drones.
        initial_rpys: ndarray | None, optional
            (NUM_DRONES, 3)-shaped array containing the initial orientations of the drones (in radians).
        pyb_freq : int, optional
            The frequency at which PyBullet steps (a multiple of ctrl_freq).
        ctrl_freq : int, optional
            The frequency at which the environment steps.
        gui : bool, optional
            Whether to use PyBullet's GUI.
        vision : bool, optional
            Whether to use the camera/vision sensor
        img_res: numpy array, optional
            Specify the resolution of the captured images
        save_imgs : bool, optional
            Whether to save pictures.
        obstacle_ids : list, optional
            Select how many and which obstacles/assets to add to the simulation
        labyrinth_id : string, optional
            Select the type of labyrinth to insert
        sensors_attributes : bool, optional
            mettere True se ci sono i RangeFinders
        max_sensors_range : float
            gittata RangeFinders (TODO CONTROLLARE SIA IN METRI)
        TODO: Aggiungere help delle variabili dopo max_sensors_range
        """        

        super().__init__(drone_model=drone_model,
                         num_drones=num_drones,
                         neighbourhood_radius=neighbourhood_radius,
                         initial_xyzs=initial_xyzs,
                         initial_rpys=initial_rpys,
                         physics=Physics.PYB,
                         pyb_freq=pyb_freq,
                         ctrl_freq=ctrl_freq,
                         gui=gui,
                         vision=vision,
                         img_res=img_res,
                         save_imgs=save_imgs,
                         obstacle_ids=obstacle_ids,
                         labyrinth_id=labyrinth_id,
                         sensors_attributes=sensors_attributes,
                         max_sensors_range=max_sensors_range,
                         output_folder=output_folder
                         )
        
        self.DIST_WALL_REF = ref_distance  # distanza di riferimento dal muro
        self.S_WF = np.array([[s_WF] for j in range(self.NUM_DRONES)])  # wall following side: 1 for wall on Right side, -1 for wall on Left Side
        self.C_OMEGA = c_omega  # costante di velocità di correzione rotta. unica per tutti i droni
        self.C_VEL=c_vel        # output sulla velocità di controllo per il raddrizzamento
        self.WFSTATE = np.array([[WFstate] for j in range(self.NUM_DRONES)])
        self.PREV_DIST = np.array([[] for j in range(self.NUM_DRONES)])

    ################################################################################

    def NextWP(self): #Aggiungere gli input necessari
        """Definisce la logica di navigazione trovando il prossimo WP usando le funzioni:
        - _WallFollowing()
        - _WallFollowingAndAlign()
        - _DecisionSystem
        
        """

        #TODO: definire funzione

        pass

    ################################################################################

    def _decisionSystem(self): #Aggiungere gli input necessari
        """Definisce la logica di scelta della direzione da prendere dopo uno stop
        
        """

        #TODO: Definire funzione

        pass
    
    ################################################################################

    def _WallFollowing(self,WFstate):     #ALG B.1
        """Funzione per seguire in modo allineato il muro -> tira fuori la rotazione desiderata da mandare ai controlli
        # TODO check gli help
        
        Parameters
        ---------
        WFstate prende tre possibili valori:
        - WFstate = 0 ruotare per allinearsi al muro
        - WFstate = 1 ruotare e seguire il muro
        - WFstate = 2 ruotare attorno all'angolo
       
        Returns
        --------
        omega: tipo
            descrizione
        vel: tipo
            descrizione
        WFstate: int
            aggiorna il WFstate
        """
        td = 0.02           ## threshold distance to determine wether the drone is almost at reference distance from wall
        sWF = self.S_WF     ## mi sa che tocca sia un vettore con un valore per ogni drone
        hit_distance,Hit_point,self.Min_dist= self._MinDistToWall(self)
        cw = self.C_OMEGA   ## mi sa che tocca sia un vettore con un valore per ogni drone
        cv = self.C_VEL     ## mi sa che tocca sia un vettore con un valore per ogni drone
        self.beta = np.deg2rad(90 - self.alfa/2) #angolo tra rf e rs 
        omega = []
        vel = []
        self.rf = [] ## front distance to hit (observation)
        self.rs = [] ##
        for i in range(self.NUM_DRONES) :
            for j in range(self.NUM_SENSORS) :
                if hit_distance[i][1]!= self.max_range : 
                    self.rf.append(hit_distance[j][1]) # due range finders laterai
                    self.rs.append(hit_distance[j][21])
                elif hit_distance[j][20]!=self.max_range :
                    self.rf.append(hit_distance [j][20])  #raggi esterni stereocamera/sensore di profondità 
                    self.rs.append(hit_distance[j][21])
                else :
                    self.rf.append([self.max_range])
                    self.rs.append([self.max_range])
                dR = self.Min_dist #minima distanza dal muro calcolata con RANSAC
                if WFstate == 0 :#ruoto per allinearmi al muro
                    omega.append([-1*sWF[j]*cw])
                    vel.append([0])
                    if np.abs(self.rs - self.rf * np.cos(self.beta)) < td :
                        WFstate = 1 # mi sono ruotato e ora voglio seguire il muro
                    if self.rf == self.max_range :
                        WFstate = 2 #mi giro ma non vedo più il muro quindi ci sta un angolo
                elif WFstate == 1 :#ruotare e seguire il muro
                    vel.append([cv])
                    Omega = self._WallFollowingandAllign(self)
                    omega.append(Omega)
                    if dR < self.DIST_WALL_REF :
                        WFstate = 0 #sono vicino al muro devo girare per allinearmi
                    if self.rf == self.max_range :
                        WFstate = 2 #dovrei seguire il muro ma non lo vedo più --> ci sta un angolo
                elif WFstate == 2 :
                    vel.append([cv])
                    omega.append([sWF[j]*cv/self.DIST_WALL_REF])
                if np.abs(self.rs - self.rf* np.cos(self.beta)) < 0.01 :
                        WFstate = 1 # ho agirato l'angolo e conttinuo a seguire il muro
                if dR < self.DIST_WALL_REF:
                        WFstate = 0 #sto girando vedo il muro ma non sono allineato, mi devo girare    
        
        return omega , vel , WFstate
    
    def _WallFollowing2(self,WFstate):    #ALG B.1
        """Funzione per seguire in modo allineato il muro -> tira fuori la rotazione desiderata da mandare ai controlli
        # TODO check gli help
        
        Parameters
        ---------
        WFstate (NUM_DRONES, 1)-shaped  prende tre possibili valori:
        - WFstate = 0 ruotare da fermo (v=0) per allinearsi al muro
        - WFstate = 1 WF vero e proprio (v!=0) e piccoli aggiustamenti di rotta con _WallFollowingandAlign()
        - WFstate = 2 ruotare attorno all'angolo
       
        Returns
        --------
        omega: tipo
            descrizione
        vel: tipo
            descrizione
        WFstate: int
            aggiorna il WFstate
        """
        td = 0.02           ## threshold distance to determine wether the drone is almost at reference distance from wall
        sWF = self.S_WF     ## implementato come (NUM_DRONES, 1)-shaped array che indica muro a destra o sinistra
        observation , distance = self._MinDistToWall()
        cw = self.C_OMEGA   
        cv = self.C_VEL     
        omega = np.array([[] for j in range(self.NUM_DRONES)])
        vel = np.array([[] for j in range(self.NUM_DRONES)])
        
        for i in range(self.NUM_DRONES) :
            
            # dist ? = ...
            if self.WFSTATE[i] == 0 : #ruoto per allinearmi al muro
                omega [i] = ([-1*sWF[i]*cw])
                vel [i] = ([0])
                if np.abs(self.PREV_DIST[i] - distance[i]) < td :
                    self.WFSTATE[i]= 1 # mi sono ruotato e ora voglio seguire il muro
                if sWF == 1: # wallfollowing con muro a destra
                    if observation[i][3] == self.max_range: 
                        self.WFSTATE[i] = 2 #mi giro ma non vedo più il muro quindi ci sta un angolo
                elif sWF == -1: # wallfollowing con muro a sinistra
                    if observation[i][1] == self.max_range:
                        self.WFSTATE[i] = 2 #mi giro ma non vedo più il muro quindi ci sta un angolo
            elif self.WFSTATE[i] == 1 :#ruotare e seguire il muro
                vel [i] = ([cv]) 
                Omega = self._WallFollowingandAlign2()  # devo farglielo fare al singolo drone?  forse serve mettere nth drone
                omega[i]= ([Omega])
                if dR < self.DIST_WALL_REF :
                    self.WFSTATE[i] = 0 #sono vicino al muro devo girare per allinearmi
                if self.rf == self.max_range :
                    self.WFSTATE[i] = 2 #dovrei seguire il muro ma non lo vedo più --> ci sta un angolo
            elif self.WFSTATE[i] == 2 :
                vel.append([cv])
                omega.append([sWF[j]*cv/self.DIST_WALL_REF])
            if np.abs(self.rs - self.rf* np.cos(self.beta)) < 0.01 :
                    self.WFSTATE[i] = 1 # ho agirato l'angolo e conttinuo a seguire il muro
            if dR < self.DIST_WALL_REF:
                    self.WFSTATE[i] = 0 #sto girando vedo il muro ma non sono allineato, mi devo girare    
        
        return omega , vel , WFstate
    ################################################################################

    def _WallFollowingandAlign(self):     #ALG B.2
        td = 0.02
        omega=[]
        for j in range(self.NUM_DRONES):
            if np.abs(self.distWallRef-self.Min_dist[j]) > td :#sono lontano da td
                if self.distWallRef-self.Min_dist[j] > td :
                    omega.append([self.sWF*self.comega]) # molto lontano dal muro
                else : 
                    omega.append([-self.sWF*self.comega]) #troppo vicino al muro
            elif np.abs(self.distWallRef-self.Min_dist[j]) < td :#sono vicino al td
                if self.rs[j] > self.rf[j]*np.cos(self.beta):
                      omega.append([self.sWF*self.comega])
                else : 
                     omega.append([-self.sWF*self.comega])
            else :
                 omega.append([0])
        return omega
    
    def _WallFollowingandAlign2(self):    #ALG B.2  versione con solo il sensore davanti, credo necessiti di un meccanismo di memoria
        """funzione per seguire il muro -> mi da la omega mentre sto navigando vicino al muro per evitare di allontanarmi dal muro
        Returns
        --------
        omega: tipo
            descrizione
        vel: tipo
            descrizione
        WFstate: int
            aggiorna il WFstate
        """
        td = 0.02
        omega=np.array([[0] for i in range(self.NUM_DRONES)] )
        observation , distance = self._MinDistToWall()
        sWF = self.S_WF

        
        for i in range(self.NUM_DRONES):
            if np.abs(self.DIST_WALL_REF-distance[i]) > td :# sono lontano dalla distanza desiderata
                if self.DIST_WALL_REF-distance[i] > td :    # troppo lontano dal muro
                    omega[i] = ([self.S_WF*self.C_OMEGA])        # TODO check sign
                else :                                           # troppo vicino al muro
                    omega[i] = ([-self.S_WF*self.C_OMEGA])       
            elif np.abs(self.DIST_WALL_REF-self.Min_dist[i]) < td :#sono vicino alla distanza desiderata
                # meccanismo di fine tune alignment basato su confronto con self.PREV_DIST
                if self.PREV_DIST[i] > distance[i]:
                    omega[i] = ([self.S_WF*self.C_OMEGA])
                else : 
                    omega[i] = ([-self.S_WF*self.C_OMEGA])
            else :
                omega[i] = ([0])
        return omega    
    
    ################################################################################

    def _MinDistToWall(self):
        """Distanza minima dall'ostacolo.
            ATTENZIONE: nelle fasi di volo circa parallelo al muro non ritorna la distanza minima esatta ma l'observation del RF laterale
        TODO richiede sWF per capire da che lato  (e faccio che la aggiorna???)
        Output
        ----------
        observation : ndarray
            (NUM_DRONES, 4)-shaped array containing the front/right/back/left distance observed
        distance : ndarray
            (NUM_DRONES, 1)-shaped array containing the minimal distance observed  
        self.PREV_DIST: ndarray

        Updates
        ----------
            (NUM_DRONES, 1)-shaped array, copia di distance che vorrei fosse salvata a livello di script topo.py e che entra nel seguente run di NextWP
        """
        observation, Hit_point = self._sensorsObs()
        sWF = self.S_WF
        distance=np.array([[np.inf] for j in range(self.NUM_DRONES)] )   
        for i in range(self.NUM_DRONES):
            distance[i]=[]
            rF = observation[i][0] # distanza frontale
            rL = observation[i][1] # distanza sinistra
            rB = observation[i][2] # distanza retro
            rR = observation[i][3] # distanza destra
            if sWF == 1: # se wall following con muro sulla mia destra , devo trovare le distanze N-E e S-E
                NEalfa = np.arctan(rF/rR) # complementare dell'angolo di "raddrizzamento" rispetto al muro
                SEalfa = np.arctan(rB/rR) # stessa ma dietro
                NEdistance = rF * np.cos(NEalfa)
                SEdistance = rB * np.cos(SEalfa)
                distance[i] = min(NEdistance, SEdistance) # minimo tra le due distanze
                        ### (una in genere è inf a meno che non siamo vicino a uno spigolo e posso sfruttare ciò)
            elif sWF == -1: # se wall following con muro sulla mia sinistra , devo trovare le distanze N-O e S-O
                NWalfa = np.arctan(rF/rL) #complementare dell'angolo di "raddrizzamento" rispetto al muro
                SWalfa = np.arctan(rB/rL) #complementare dell'angolo di "raddrizzamento" rispetto al muro
                NWdistance = rF * np.cos(NWalfa)
                SWdistance = rB * np.cos(SWalfa)
                distance[i] = min(NWdistance, SWdistance) #distanza perpendicolare che però potrebbe essere vuota per la questione OoR
            if distance[i] == np.inf:  # se sia front che retro è OoR nel lato del wallfollowing...
                if sWF == 1:
                    distance[i] = rR   # ... prendo l'observation laterale
                elif sWF == -1:
                    distance[i] = rL   # ... prendo l'observation laterale
            self.PREV_DIST[i] = distance

        return observation , distance

        '''
        for i in range(self.NUM_DRONES):
            for j in range(observation[i][j]):
                if observation[i][j] < self.max_range:
                   hit_point = Hit_point[i][j]
            if hit_point == []:
                distance.append ([])
            else : 
                idx = np.random.choice(len(hit_point), 2, replace=False)
                x1, y1 = hit_point[idx[0]][0:1]
                x2, y2 = hit_point[idx[1]][0:1]

                # Calcola l'equazione della retta usando i due punti
                slope = (y2 - y1) / (x2 - x1)
                intercept = y1 - slope * x1

                # Valuta la distanza da ogni punto alla retta
                distance.append([np.abs(slope * x1 - y1 + intercept) / np.sqrt(slope**2 + 1)])
        
        return observation, Hit_point , distance
        '''