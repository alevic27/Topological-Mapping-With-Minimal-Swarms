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
                 physics: Physics = Physics.PYB,
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
                 s_WF: int = +1,
                 c_omega : float = 0.1,
                 c_vel: float = 0.1,
                 output_folder='results',
                 WFstate : int = 1,
                 td : float = 0.02,
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
                         physics= physics,
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
        self.C_VEL = c_vel        # output sulla velocità di controllo per il raddrizzamento
        self.WFSTATE = np.array([[WFstate] for j in range(self.NUM_DRONES)])
        # self.PREV_DIST = np.array([[] for j in range(self.NUM_DRONES)]) # obsolete
        self.CTRL_FREQ = ctrl_freq 
        self.MAX_RANGE = max_sensors_range
        self.distance = np.array([[np.inf , np.inf] for j in range(self.NUM_DRONES)] )
        self.prev_rR = np.array([[self.MAX_RANGE] for j in range(self.NUM_DRONES)] )
        self.prev_rL = np.array([[self.MAX_RANGE] for j in range(self.NUM_DRONES)] )
        self.turn_start_yaw = np.array([[0.] for j in range(self.NUM_DRONES)] )
        self.wfstatezero_debugger = np.array([[0. , np.inf , 0.] for j in range(self.NUM_DRONES)] )
        ## vettore di 3 numeri, 
        # il primo è un counter angolare che si accerta che al massimo faccia un giro
        # il secondo è il valore della distanza frontale minima
        ###################################### (se questa è maggiore di DISTREF si entrerà in state -1)
        ###################################### (se è minore di DISTREF tocca inventare uno stato di allontanamento ortogonale)
        # il terzo si aggiorna insieme al secondo e indica la direzione della distanza minima in qualche modo
        self.state1counter = np.array([[0] for j in range(self.NUM_DRONES)] )
        self.state4counter = np.array([[0] for j in range(self.NUM_DRONES)] )
        self.td = td ## threshold distance to determine wether the drone is almost at reference distance from wall
        self.WF_ref_angle = np.array([[0.] for j in range(self.NUM_DRONES)] ) # yaw di riferimento di start dello stato 1 per non allontanarsi troppo dalla direzione parallela

        ## database init
        self.drones_db = {} 
        self.adjacency_matrices = []
    ################################################################################

    def NextWP(self,obs,observation): #Aggiungere gli input necessari
        """Definisce la logica di navigazione trovando il prossimo WP usando le funzioni:
        - _DecisionSystem
        - _WallFollowing()  , il quale chiama _WallFollowingAndAlign()
        """
        self.observation = observation
        TARGET_POS = np.array([[0. , 0., 0.]for j in range(self.NUM_DRONES)])
        TARGET_RPY = np.array([[0. , 0., 0.]for j in range(self.NUM_DRONES)])
        time_step  = 1 / self.CTRL_FREQ

        TARGET_VEL = np.array([[0. , 0., 0.]for j in range(self.NUM_DRONES)])
        TARGET_RPY_RATES = np.array([[0. , 0., 0.]for j in range(self.NUM_DRONES)])
        #self.S_WF=self._decisionSystem()  # controllare se aggiornare in self o no
        TARGET_OMEGA, RELATIVE_FRAME_VEL , self.WFSTATE = self._WallFollowing3()
        for i in range(self.NUM_DRONES) :
            yaw = obs[i][9]
            print("absolute yaw =", np.degrees(yaw),"°")
            rot_mat = np.array([[np.cos(yaw) ,-np.sin(yaw) , 0.0],  
                                [np.sin(yaw) , np.cos(yaw) , 0.0],                                           
                                [-1.0        , 0.0         , 1.0]])
            ABSOLUTE_FRAME_VEL = np.dot(RELATIVE_FRAME_VEL[i], rot_mat.T )
            TARGET_POS[i][0] = obs[i][0] + ABSOLUTE_FRAME_VEL[0] * time_step  
            TARGET_POS[i][1] = obs[i][1] + ABSOLUTE_FRAME_VEL[1] * time_step 
            TARGET_POS[i][2] = obs[i][2] #+ ABSOLUTE_FRAME_VEL[2] * time_step 

            TARGET_RPY[i][0] = obs[i][7]
            TARGET_RPY[i][1] = obs[i][8]   
            TARGET_RPY[i][2] = obs[i][9] #+ TARGET_OMEGA[i] * time_step # solo il terzo elemento

            TARGET_VEL[i][0] = ABSOLUTE_FRAME_VEL[0]
            TARGET_VEL[i][1] = ABSOLUTE_FRAME_VEL[1]
            TARGET_VEL[i][2] = 0.

            TARGET_RPY_RATES[i][0] = 0.
            TARGET_RPY_RATES[i][1] = 0.
            TARGET_RPY_RATES[i][2] = TARGET_OMEGA[i]

        for j in range(self.NUM_DRONES) : # saves actual mindist to distance[0]. It will be the prev_dist of next step
            self.distance[j][0] =  self.distance[j][1]
        return TARGET_POS, TARGET_RPY, TARGET_VEL, TARGET_RPY_RATES
        
    ################################################################################

    def _decisionSystem(self): #Aggiungere gli input necessari
        """Definisce la logica di scelta della direzione da prendere dopo uno stop (dopo che becca muro)
           #TODO: Definire funzione Controlla sWF e observation[i][0:3] e, capendo dov è il muro sceglie una nuova direzione
        """
        sWF = np.array([[self.S_WF] for j in range(self.NUM_DRONES)])
        
        pass
    
    ################################################################################

    def _WallFollowing2(self):    #ALG B.1
        """Funzione per seguire in modo allineato il muro -> tira fuori la rotazione desiderata da mandare ai controlli
        # TODO check gli help
        
        Parameters (non input ma nel self)
        ---------
        WFstate (NUM_DRONES, 1)-shaped  prende tre possibili valori:
        - WFstate = -1 stato iniziale, avanzamento lineare finchè non incontra un muro
        - WFstate = 0 ruotare da fermo (v=0) per allinearsi al muro
        - WFstate = 1 WF vero e proprio (v!=0) e piccoli aggiustamenti di rotta con _WallFollowingandAlign()
        - WFstate = 2 ruotare attorno all'angolo
        - WFstate = 3 stato transitorio di uscita da 2 , piccolo avanzamento lineare per non perdere subito rR o rL
        - WFstate = 4 stato di debug di 0 quando vi si entra in situazioni troppo vicine o troppo lontane al muro #TODO: da sviluppare
                per ora il debug è interno a WFstate = 0
        Returns
        --------
        omega: ndarray - [[float] for j in range(self.NUM_DRONES)]
            (NUM_DRONES, 1)-shaped array che contiene la omega (yaw_rate) di controllo per ogni drone
        vel: ndarray - [[float] for j in range(self.NUM_DRONES) ]
            (NUM_DRONES, 1)-shaped array che contiene la velocita di avanzamento frontale di controllo per ogni drone
        WFstate: ndarray - [[int] for j in range(self.NUM_DRONES) ]
            (NUM_DRONES, 1)-shaped array che indica lo stato nella logica di wallfollowing per ogni drone
        """
        NWdistance,SWdistance,SEdistance,NEdistance = self._MinDistToWall2()  ## updates distance [[prev_dist , curr_dist]]  TODO: dipende da s_WF che va aggiornato con qualche funzione
        cw = self.C_OMEGA   
        cv = self.C_VEL     
        omega = np.array([[0.] for j in range(self.NUM_DRONES)])
        vel = np.array([[0.] for j in range(self.NUM_DRONES)])
        state_2_omega_coeff = 0.95  # <1 per allargare il raggio di curvatura del WFSTATE = 2
        time_step  = 1 / self.CTRL_FREQ
        print("la minima distanza da _MinDistToWall2() è ", self.distance[0][1])
        print("la differenza tra lei e quella desiderata dal muro è " ,self.distance[0][1] - self.DIST_WALL_REF)
        for i in range(self.NUM_DRONES) :
            print("er WFSTATE è",self.WFSTATE[i])
            rF = self.observation[i][0] # distanza frontale
            rL = self.observation[i][1] # distanza sinistra
            rB = self.observation[i][2] # distanza retro
            rR = self.observation[i][3] # distanza destra

        ######################  STATO -1 : AVANZAMENTO LINEARE FINO A WALLREF  ############################
        ### stato di avanzamento lineare finchè la min dist non si avvicina alla DIST_WALL_REF
            if self.WFSTATE[i] == -1 : 
                omega [i] = ([0])
                vel [i] = ([cv])
                if self.distance[i][1] - self.DIST_WALL_REF < self.td:
                    self.WFSTATE[i] = 0
                    self.wfstatezero_debugger[i] = [0. , np.inf , 0.]
                    print("esco da WFSTATE = 3 e entro in WFSTATE = 0")
                    ### scelta unica del S_WF per il drone ###
                    if self.distance[i][1] in [rL, NWdistance, SWdistance]:
                        self.S_WF[i] = -1
                    elif self.distance[i][1] in [rR, NEdistance, SEdistance]:
                        self.S_WF[i] = 1
                    else:
                        self.S_WF[i] = 1  # preferenza antioraria                

        ######################  STATO 0 : GIRO SUL POSTO PER ALLINEARSI AL MURO  ############################
        ### note: funzionante solo a DISTWALLREF dal muro
        ### possibili migliorie: - aumentare la precisione di ripartenza da angolo
        ###                      - il debug funge solo se rispetta le condizioni al primo giro, poi va all'infinito
            if self.WFSTATE[i] == 0 : 
                omega [i] = ([+1*self.S_WF[i][0]*cw  *   2])   
                vel [i] = ([0])                
                if self.S_WF == 1: # wallfollowing con muro a destra
                    print("differenza tra range destro e dist di riferimento: ",np.abs(rR - self.DIST_WALL_REF))
                    print("differenza tra range destro e range destro precedente: ",np.abs(rR - self.prev_rR[i][0]))
                    if np.abs(rB - self.DIST_WALL_REF) < self.td and np.abs(rR - self.DIST_WALL_REF) < self.td: # se dietro e destra so circa ar top
                        if rR != self.MAX_RANGE and np.abs(rR - self.prev_rR[i][0]) < self.td*0.01 :
                            self.WFSTATE[i] = 1
                            print("esco da WFSTATE = 0 (in un angolo) e entro in WFSTATE = 1")
                    #elif np.abs(rR - self.DIST_WALL_REF) < self.td :   # /6 o *0.17  è lo sweet spot con rR 
                    #    self.WFSTATE[i] = 1
                    #    print("esco da WFSTATE = 0 (non causa angolo) e entro in WFSTATE = 1")
                    ### provo a farlo ripartire da 0 in uno se rR e prev_rR sono molto simili
                    elif rR != self.MAX_RANGE and np.abs(rR - self.prev_rR[i][0]) < self.td*0.01 :  
                        self.WFSTATE[i] = 1
                        print("esco da WFSTATE = 0 (non causa angolo) e entro in WFSTATE = 1")
                elif self.S_WF == -1: # wallfollowing con muro a sinistra                    
                    print("differenza tra range sinistro e dist di riferimento è ",np.abs(rL - self.DIST_WALL_REF))
                    print("differenza tra range sinistro e range sinistro precedente: ",np.abs(rL - self.prev_rL[i][0]))
                    if np.abs(rB - self.DIST_WALL_REF) < self.td and np.abs(rL - self.DIST_WALL_REF) < self.td: # se dietro e sinistra so circa ar top
                        self.WFSTATE[i] = 1
                        print("esco da WFSTATE = 0 (in un angolo) e entro in WFSTATE = 1")
                    elif rL != self.MAX_RANGE and np.abs(rL - self.prev_rL[i][0]) < self.td*0.01 :    
                        self.WFSTATE[i] = 1
                        print("esco da WFSTATE = 0 (non causa angolo) e entro in WFSTATE = 1")
        ######### 0-STATE debugger, starta solo se non sono uscito da 0 per nessun altro motivo #####
                self.wfstatezero_debugger[i][0] += omega[i] * time_step
                print(self.wfstatezero_debugger[i][0])
                if abs(self.wfstatezero_debugger[i][0]) < 2*np.pi : # ancora non giro completo
                    if rF < self.wfstatezero_debugger[i][1]: # se il mio front-range è minore del minore rolevato finora
                        self.wfstatezero_debugger[i][1] = rF # ci metto la distanza frontale
                        self.wfstatezero_debugger[i][2] = self.wfstatezero_debugger[i][0]
                else: 
                    if self.wfstatezero_debugger[i][1] > self.DIST_WALL_REF : # lontano
                        if np.abs( (self.wfstatezero_debugger[i][0] - (2*np.pi) ) - self.wfstatezero_debugger[i][2] ) < self.td*2 : 
                            self.WFSTATE[i] = -1
                            print("esco da WFSTATE = 0 per DEBUG troppo lontano e entro in WFSTATE = -1")
                    elif self.wfstatezero_debugger[i][1] < self.DIST_WALL_REF: #vicino
                        if np.abs( (self.wfstatezero_debugger[i][0]- (2*np.pi) ) - (self.wfstatezero_debugger[i][2] - (np.pi) ))  < self.td*2 :
                            self.WFSTATE[i] = -1
                            print("esco da WFSTATE = 0 per DEBUG troppo vicino e entro in WFSTATE = -1")
                    else:
                        print("ERROR NON DEBUGGA LO STATO 0")

        ######################  STATO 1 : WALLFOLLOWING  ############################
        ### possibili migliorie: - tuning dei coefficienti di _WallFollowingandAlign per un andamento meno oscillante
            elif self.WFSTATE[i] == 1 : # wallfollowing
                vel [i] = ([cv]) 
                omega[i] = self._WallFollowingandAlign(i)  
                if self.S_WF == 1: # wallfollowing con muro a destra
                    if np.abs(rF - self.DIST_WALL_REF) < 5*self.td and np.abs(rR - self.DIST_WALL_REF) < 5*self.td :
                        self.WFSTATE[i] = 0
                        self.wfstatezero_debugger[i] = [0. , np.inf , 0.]
                        print("esco da WFSTATE = 1 e entro in WFSTATE = 0 visto che sono in un angolo")
                    #if rR > self.DIST_WALL_REF + 5*self.td:
                    if np.abs(self.prev_rR[i][0] - rR) > 1*self.td: #condizione con la storia di rR
                        self.WFSTATE[i] = 2 #dovrei seguire il muro ma non lo vedo più --> ci sta un angolo 
                        print("esco da WFSTATE = 1 e entro in WFSTATE = 2")
                        self.turn_start_yaw[i][0] = 0
                elif self.S_WF == -1: # wallfollowing con muro a sinistra
                    if np.abs(rF - self.DIST_WALL_REF) < 5*self.td and np.abs(rL - self.DIST_WALL_REF) < 5*self.td :
                        self.WFSTATE[i] = 0
                        self.wfstatezero_debugger[i] = [0. , np.inf , 0.]
                        print("esco da WFSTATE = 1 e entro in WFSTATE = 0 visto che sono in un angolo")
                    #if rL > self.DIST_WALL_REF + 5*self.td:
                    if np.abs(self.prev_rL[i][0] - rL) > 1*self.td: #condizione con la storia di rR
                        self.WFSTATE[i] = 2 #dovrei seguire il muro ma non lo vedo più --> ci sta un angolo 
                        print("esco da WFSTATE = 1 e entro in WFSTATE = 2")
                        self.turn_start_yaw[i][0] = 0
                        
        ######################  STATO 2 : CURVA  ############################
        ### possibili migliorie: - rimuovere la cosa che dopo va in 3 e farlo andare in 0
            elif self.WFSTATE[i] == 2 : 
                vel [i] = ([cv])
                omega[i] = ([-self.S_WF[i][0]*state_2_omega_coeff*cv/self.DIST_WALL_REF])
                print("la differenza tra distance e DISTWALLREF",  np.abs(self.distance[i][1] - self.DIST_WALL_REF) )
                self.turn_start_yaw[i][0] += omega[i] * time_step  
                print ("l'angolo di curva accumulato è" , self.turn_start_yaw[i][0])
                if np.abs(self.turn_start_yaw[i][0]) > 1 : # se l'angolo accumulato è maggiore di un certo angolo (per ora metto 1 che è tipo 60 deg)
                    # e solo ora faccio la verifica con grosso salto di rR # TODO sensibilità da tunare
                    if self.S_WF == 1: # wallfollowing con muro a destra  
                        if self.prev_rR[i][0] != self.MAX_RANGE and np.abs(rR - self.prev_rR[i][0]) > self.td*2.5 : 
                            self.WFSTATE[i] = 3
                            print("esco da WFSTATE = 2 e entro in WFSTATE = 3")
                    elif self.S_WF == -1: # wallfollowing con muro a sinistra
                        if self.prev_rL[i][0] != self.MAX_RANGE and np.abs(rL - self.prev_rL[i][0]) > self.td*2.5 :
                            self.WFSTATE[i] = 3
                            self.state4counter[i][0] = 0
                            print("esco da WFSTATE = 2 e entro in WFSTATE = 3")

        ######################  STATO 3 : USCITA DALLA CURVA  ############################
        ### note: per ora serve a far sì che faccia qualche step paralleli al muro così che non perde subito il range laterale
        ### possibili migliorie: - rimuovere la omega post 20 step
        ###                      - tarare il numero step
        ###                      - rimuovere questo stato completamente e fare che va diretto in 0 (poco gradevole)
            elif self.WFSTATE[i] == 3: 
                vel [i] = ([cv])
                omega[i] = ([0]) # ([-self.S_WF[i][0]*cw*0.1]) # prova a mettere una piccola omega qua
                self.state3counter[i][0] += 1
                print(self.state3counter[i][0])
                if self.state3counter[i][0] > 20 :
                    omega[i] = self._WallFollowingandAlign(i)
                    if self.S_WF == 1 and abs(min(rF,rR, NEdistance) - self.DIST_WALL_REF) < self.td: # accresci la sensibilità        
                        self.WFSTATE[i] = 0
                        self.wfstatezero_debugger[i] = [0. , np.inf , 0.]
                        print("esco da WFSTATE = 3 e entro in WFSTATE = 0 ")
                    if self.S_WF == -1 and abs(min(rF,rL, NWdistance) - self.DIST_WALL_REF) < self.td: # accresci la sensibilità        
                        self.WFSTATE[i] = 0
                        self.wfstatezero_debugger[i] = [0. , np.inf , 0.]
                        print("esco da WFSTATE = 3 e entro in WFSTATE = 0 ")

            print("omega=",omega[i][0])
            print("nun mollà drone") 
            self.prev_rR[i] = rR
            self.prev_rL[i] = rL
            ###### manual override settings ######
            #self.WFSTATE[i] = 0
            #omega[i] = ([0.5])
            #vel [i] = ([0.2])            
        return omega , vel , self.WFSTATE
    
    ################################################################################

    def _WallFollowing3(self):    #ALG B.1
        """Funzione per seguire in modo allineato il muro -> tira fuori la rotazione desiderata da mandare ai controlli
        ### REMAKE DEL 2 sfruttando il movimento laterale
        
        Parameters (non input ma nel self)
        ---------
        WFstate (NUM_DRONES, 1)-shaped  prende tre possibili valori:
        - WFstate = -1 stato iniziale, avanzamento lineare finchè non incontra un muro
        - WFstate = 0 ruotare da fermo (v=0) per allinearsi al muro
        - WFstate = 1 WF vero e proprio (v!=0) e piccoli aggiustamenti di rotta con _WallFollowingandAlign()
        - WFstate = 2 ruotare attorno all'angolo
        - WFstate = 3 stato transitorio di uscita da 2 , piccolo avanzamento lineare per non perdere subito rR o rL
        - WFstate = 4 stato di debug di 0 quando vi si entra in situazioni troppo vicine o troppo lontane al muro #TODO: da sviluppare
                per ora il debug è interno a WFstate = 0
        Returns
        --------
        omega: ndarray - [[float] for j in range(self.NUM_DRONES)]
            (NUM_DRONES, 1)-shaped array che contiene la omega (yaw_rate) di controllo per ogni drone
        vel: ndarray - [[float float float] for j in range(self.NUM_DRONES) ]
            (NUM_DRONES, 3)-shaped array che contiene la velocita di controllo per ogni drone
        WFstate: ndarray - [[int] for j in range(self.NUM_DRONES) ]
            (NUM_DRONES, 1)-shaped array che indica lo stato nella logica di wallfollowing per ogni drone
        """
        NWdistance,SWdistance,SEdistance,NEdistance = self._MinDistToWall2()  ## updates distance [[prev_dist , curr_dist]]  
        cw = self.C_OMEGA   
        cv = self.C_VEL     
        omega = np.array([[0.] for j in range(self.NUM_DRONES)])
        vel = np.array([[0. , 0. , 0.] for j in range(self.NUM_DRONES)])
        state_2_omega_coeff = 0.95  # <1 per allargare il raggio di curvatura del WFSTATE = 2
        time_step  = 1 / self.CTRL_FREQ
        for i in range(self.NUM_DRONES) :
            print("er WFSTATE è",self.WFSTATE[i])
            rF = self.observation[i][0] # distanza frontale
            rL = self.observation[i][1] # distanza sinistra
            rB = self.observation[i][2] # distanza retro
            rR = self.observation[i][3] # distanza destra

        ######################  STATO -1 : AVANZAMENTO LINEARE FINO A WALLREF  ############################
        ### stato di avanzamento lineare finchè la min dist non si avvicina alla DIST_WALL_REF
            if self.WFSTATE[i] == -1 : 
                omega [i] = ([0])
                vel [i] = np.dot(  cv , [1. , 0. , 0.] )
                if self.distance[i][1] - self.DIST_WALL_REF < self.td:
                    self._SwitchWFSTATE(i, 0)
                    self.wfstatezero_debugger[i] = [0. , np.inf , 0.]
                    print("esco da WFSTATE = 3 e entro in WFSTATE = 0")
                    ### scelta unica del S_WF per il drone ###
                    if self.distance[i][1] in [rL, NWdistance, SWdistance]:
                        self.S_WF[i] = -1
                    elif self.distance[i][1] in [rR, NEdistance, SEdistance]:
                        self.S_WF[i] = 1
                    else:
                        self.S_WF[i] = 1  # preferenza antioraria                

        ######################  STATO 0 : GIRO SUL POSTO PER ALLINEARSI AL MURO  ############################
        ### note: l'uscita da un angolo convesso è funzionante solo a DISTWALLREF dal muro
        ### possibili migliorie: - aumentare la precisione di ripartenza da angolo
        ###                      - il debug funge solo se rispetta le condizioni al primo giro, poi va all'infinito
            if self.WFSTATE[i] == 0 : 
                omega [i] = ([+1*self.S_WF[i][0]*cw  *   2])   
                vel [i] = ([0. , 0. , 0.])                
                if self.S_WF == 1: # wallfollowing con muro a destra
                    print("differenza tra range destro e range destro precedente: ",np.abs(rR - self.prev_rR[i][0]))
                    if np.abs(rB - self.DIST_WALL_REF) < self.td and np.abs(rR - self.DIST_WALL_REF) < self.td: # se dietro e destra so circa ar top
                        if rR != self.MAX_RANGE and np.abs(rR - self.prev_rR[i][0]) < self.td*0.01 : # TODO: aggiusta sensibilità
                            self._SwitchWFSTATE(i, 1)
                            self.state1counter[i][0] = 0
                            self.WF_ref_angle[i] = self.rpy[i][2]
                            print("esco da WFSTATE = 0 (in un convesso) e entro in WFSTATE = 1")
                    elif rR != self.MAX_RANGE and np.abs(rR - self.prev_rR[i][0]) < self.td*0.01 :  
                        self._SwitchWFSTATE(i, 3)
                        print("esco da WFSTATE = 0 (causa parallelo al muro destro) e entro in WFSTATE = 3")
                elif self.S_WF == -1: # wallfollowing con muro a sinistra                    
                    print("differenza tra range sinistro e range sinistro precedente: ",np.abs(rL - self.prev_rL[i][0]))
                    if np.abs(rB - self.DIST_WALL_REF) < self.td and np.abs(rL - self.DIST_WALL_REF) < self.td: # se dietro e sinistra so circa ar top
                        if rL != self.MAX_RANGE and np.abs(rL - self.prev_rL[i][0]) < self.td*0.01 : # TODO: aggiusta sensibilità
                            self._SwitchWFSTATE(i, 1)
                            self.state1counter[i][0] = 0
                            self.WF_ref_angle[i] = self.rpy[i][2]
                            print("esco da WFSTATE = 0 (in un convesso) e entro in WFSTATE = 1")
                    elif rL != self.MAX_RANGE and np.abs(rL - self.prev_rL[i][0]) < self.td*0.01 :    
                        self._SwitchWFSTATE(i, 3)
                        print("esco da WFSTATE = 0 (causa parallelo al muro destro) e entro in WFSTATE = 3")
        ######### 0-STATE debugger, starta solo se non sono uscito da 0 per nessun altro motivo #####
        ######### TODO: da rifare con lo spost laterale
                self.wfstatezero_debugger[i][0] += omega[i] * time_step
                print("angolo di debug in gradi:" , np.degrees(self.wfstatezero_debugger[i][0]),"°")
                if abs(self.wfstatezero_debugger[i][0]) < 2*np.pi : # ancora non giro completo
                    if rF < self.wfstatezero_debugger[i][1]: # se il mio front-range è minore del minore rolevato finora
                        self.wfstatezero_debugger[i][1] = rF # ci metto la distanza frontale
                        self.wfstatezero_debugger[i][2] = self.wfstatezero_debugger[i][0]
                else: 
                    if self.wfstatezero_debugger[i][1] > self.DIST_WALL_REF : # lontano
                        if np.abs( (self.wfstatezero_debugger[i][0] - (2*np.pi) ) - self.wfstatezero_debugger[i][2] ) < self.td*2 : 
                            self._SwitchWFSTATE(i, -1)
                            print("esco da WFSTATE = 0 per DEBUG troppo lontano e entro in WFSTATE = -1")
                    elif self.wfstatezero_debugger[i][1] < self.DIST_WALL_REF: #vicino
                        if np.abs( (self.wfstatezero_debugger[i][0]- (2*np.pi) ) - (self.wfstatezero_debugger[i][2] - (np.pi) ))  < self.td*2 :
                            self._SwitchWFSTATE(i, -1)
                            print("esco da WFSTATE = 0 per DEBUG troppo vicino e entro in WFSTATE = -1")
                    else:
                        print("ERROR NON DEBUGGA LO STATO 0")

        ######################  STATO 1 : WALLFOLLOWING  ############################
        ### possibili migliorie: - tuning dei coefficienti di _WallFollowingandAlign per un andamento meno oscillante
            elif self.WFSTATE[i] == 1 : # wallfollowing
                vel [i] = np.dot(  cv , [1. , 0. , 0.] ) 
                omega[i] = self._WallFollowingandAlign2(i)  
                self.state1counter[i][0] += 1
                if self.S_WF == 1: # wallfollowing con muro a destra
                    if np.abs(rF - self.DIST_WALL_REF) < 3*self.td and np.abs(rR - self.DIST_WALL_REF) < 3*self.td :
                        self._SwitchWFSTATE(i, 0)
                        self.wfstatezero_debugger[i] = [0. , np.inf , 0.]
                        print("esco da WFSTATE = 1 e entro in WFSTATE = 0 visto che sono in un angolo")
                    if np.abs(self.prev_rR[i][0] - rR) > 1.2*self.td: #condizione con la storia di rR
                        self._SwitchWFSTATE(i, 2) #dovrei seguire il muro ma non lo vedo più --> ci sta un angolo 
                        print("esco da WFSTATE = 1 e entro in WFSTATE = 2")
                        self.turn_start_yaw[i][0] = 0
                elif self.S_WF == -1: # wallfollowing con muro a sinistra
                    if np.abs(rF - self.DIST_WALL_REF) < 5*self.td and np.abs(rL - self.DIST_WALL_REF) < 5*self.td :
                        self._SwitchWFSTATE(i, 0)
                        self.wfstatezero_debugger[i] = [0. , np.inf , 0.]
                        print("esco da WFSTATE = 1 e entro in WFSTATE = 0 visto che sono in un angolo")
                    if np.abs(self.prev_rL[i][0] - rL) > 1.2*self.td: #condizione con la storia di rR
                        self._SwitchWFSTATE(i, 2) #dovrei seguire il muro ma non lo vedo più --> ci sta un angolo 
                        print("esco da WFSTATE = 1 e entro in WFSTATE = 2")
                        self.turn_start_yaw[i][0] = 0
                ###### Topological
                if self.state1counter[i][0] == 100:
                    self.add_point(i,self.pos[i],'corridor')
                    self.state1counter[i][0] = 0
                        
        ######################  STATO 2 : CURVA  ############################
        ### possibili migliorie: - rimuovere la cosa che dopo va in 3 e farlo andare in 0
            elif self.WFSTATE[i] == 2 : 
                vel [i] = np.dot(  cv , [1. , 0. , 0.] )
                omega[i] = ([-self.S_WF[i][0]*state_2_omega_coeff*cv/self.DIST_WALL_REF])
                self.turn_start_yaw[i][0] += omega[i] * time_step  
                print ("l'angolo di curva accumulato è" , self.turn_start_yaw[i][0])
                if np.abs(self.turn_start_yaw[i][0]) > 1 : # se l'angolo accumulato è maggiore di un certo angolo (per ora metto 1 che è tipo 60 deg)
                    # e solo ora faccio la verifica con grosso salto di rR # TODO sensibilità da tunare
                    if self.S_WF == 1: # wallfollowing con muro a destra
                        print("la differenza tra il range destro attuale e precedente è:", np.abs(rR - self.prev_rR[i][0]) )
                        print("la tolleranza per passare a 3 è", self.td*0.002)
                        if self.prev_rR[i][0] != self.MAX_RANGE and np.abs(rR - self.prev_rR[i][0]) < self.td*0.002 :
                            self._SwitchWFSTATE(i, 3)
                            print("esco da WFSTATE = 2 e entro in WFSTATE = 3 poichè sono abbastanza allineato col muro")
                        if rR == self.MAX_RANGE:
                            self._SwitchWFSTATE(i, 4)
                            self.state4counter[i][0] = 0
                    elif self.S_WF == -1: # wallfollowing con muro a sinistra
                        print("la differenza tra il range sinistro attuale e precedente è:", np.abs(rL - self.prev_rL[i][0]) )
                        if self.prev_rL[i][0] != self.MAX_RANGE and np.abs(rL - self.prev_rL[i][0]) < self.td*0.002 :
                            self._SwitchWFSTATE(i, 3)
                            print("esco da WFSTATE = 2 e entro in WFSTATE = 3 poichè sono abbastanza allineato col muro")

        ######################  STATO 3 : POST-0 AVVICINAMENTO A DISTWALLREF  ############################
        ### avvia dopo la curva quando il drone è circa parallelo al muro, serve a portarlo a DISTWALLREF con una velocità laterale
        ### possibili migliorie:
        ###                    
            elif self.WFSTATE[i] == 3: 
                omega[i] = ([0]) 
                vel [i] = np.dot(  cv , [1. , 0. , 0.] )
                if self.S_WF == 1: # wallfollowing con muro a destra
                    if np.abs(rR - self.DIST_WALL_REF) < self.td*0.5:
                        self._SwitchWFSTATE(i, 1)
                        self.state1counter[i][0] = 0
                        self.WF_ref_angle[i] = self.rpy[i][2]
                        print("esco da WFSTATE = 3 e entro in WFSTATE = 1 poichè sono alla dist giusta dal muro")
                    elif rR > self.DIST_WALL_REF:
                        vel[i] = np.dot(  cv , [1. , -1. , 0.] )
                    else:
                        vel[i] = np.dot(  cv , [1. , +1. , 0.] )
                elif self.S_WF == -1: # wallfollowing con muro a sinistra
                    if np.abs(rL - self.DIST_WALL_REF) < self.td*0.5:
                        self._SwitchWFSTATE(i, 1)
                        self.state1counter[i][0] = 0
                        self.WF_ref_angle[i] = self.rpy[i][2]
                        print("esco da WFSTATE = 3 e entro in WFSTATE = 1 poichè sono alla dist giusta dal muro")
                    elif rL > self.DIST_WALL_REF:
                        vel[i] = np.dot(  cv , [1. , +1. , 0.] )
                    else:
                        vel[i] = np.dot(  cv , [1. , -1. , 0.] )
        ######################  STATO 4 : FIX PER LA PERDITA DEL RANGE LATERALE  ############################
        ### avvia se durante la curva il drone vede MAXRANGE e "perde" il muro, così lo faccio avanzare un pò dritto
        ### (anche perchè dovrebbe essere circa parallelo) e poi dritto in 3 per tornare alla giusta distanza
        ### possibili migliorie:                         
            elif self.WFSTATE[i] == 4: 
                vel [i] = np.dot(  cv , [1. , 0. , 0.] )
                omega[i] = ([0]) 
                self.state4counter[i][0] += 1
                print(self.state4counter[i][0])
                if self.state4counter[i][0] > 20 :
                    self._SwitchWFSTATE(i, 3)
                    print("esco da WFSTATE = 4 e entro in WFSTATE = 3 ")   

            print("vel=", vel[i])
            print("omega=", omega[i][0]) 
            self.prev_rR[i] = rR
            self.prev_rL[i] = rL
            ###### manual override settings ######
            #self.WFSTATE[i] = 0
            #omega[i] = ([0.5])
            #vel [i] = ([0.2])            
        return omega , vel , self.WFSTATE
    
    ################################################################################
    
    def _SwitchWFSTATE(self, nth_drone, new_WFSTATE):
        """funzione per passare a un nuovo WFSTATE , sostituisce self.WFSTATE[i] = 3 
        Parameters
        ---------
        nth_drone: int
        new_WFSTATE : int
            nuovo WFSTATE a cui passare
        """
        old_WFSTATE = self.WFSTATE[nth_drone][0]  # might use to print stuff
        self.WFSTATE[nth_drone][0] = new_WFSTATE
        print(old_WFSTATE, ">>", new_WFSTATE)
        if new_WFSTATE == 1:
            self.add_point(nth_drone,self.pos[nth_drone],'corridor')
        if old_WFSTATE == 1 and new_WFSTATE == 2: # inizio curva
            self.add_point(nth_drone,self.pos[nth_drone],'junction')
        if old_WFSTATE == 2: # fine curva
            if new_WFSTATE == 0 or new_WFSTATE == 3 or new_WFSTATE == 4:
                self.add_point(nth_drone,self.pos[nth_drone],'junction')    


    def _WallFollowingandAlign(self , nth_drone):    #ALG B.2  versione con solo il sensore davanti, credo necessiti di un meccanismo di memoria
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
        outside_region_omega_reduction_factor = 1 # buono:1.5 ## prova 2
        inside_region_omega_reduction_factor = 0.8 # buono:0.4 ## TODO: aggiusta per far sì che raggiunga un'andamento bello parallelo al muro ASAP
        if np.abs(self.DIST_WALL_REF-self.distance[nth_drone][1]) > self.td : # sono fuori dalla regione accettabile ( o troppo lontano (distance > dist_wall_ref)) o troppo vicino  (distance < dist_wall_ref)
            if self.distance[nth_drone][1] > self.DIST_WALL_REF:     # troppo lontano dal muro
                omega = ([-self.S_WF[nth_drone][0]*self.C_OMEGA*outside_region_omega_reduction_factor])        # TODO check sign : CHECKED -
                print("sono fuori dalla regione e troppo lontano dal muro")
            else :                                                # troppo vicino al muro
                omega = ([+self.S_WF[nth_drone][0]*self.C_OMEGA*outside_region_omega_reduction_factor])  # +
                print("sono fuori dalla regione e troppo vicino al muro")       
        elif np.abs(self.DIST_WALL_REF-self.distance[nth_drone][1]) < self.td : #sono dentro alla regione accettabile
            # meccanismo di fine tune alignment basato su confronto con distance[iesimo drone][0]
            if self.distance[nth_drone][0] > self.distance[nth_drone][1]: # se la distanza dal muro prima era maggiore (mi sto avvicinando)
                omega = ([+self.S_WF[nth_drone][0]*self.C_OMEGA*inside_region_omega_reduction_factor])     # TODO andiamo a culo vedi se cambiare
                #omega = ([0.])
                print("sono dentro alla regione e mi sto avvicinando al muro")
            else : 
                omega = ([-self.S_WF[nth_drone][0]*self.C_OMEGA*inside_region_omega_reduction_factor])
                #omega = ([0.])
                print("sono dentro alla regione e mi sto allontanando dal muro")
              
        else :
            omega = ([0.])
        return omega      
    
    ################################################################################

    def _WallFollowingandAlign2(self , nth_drone):    #ALG B.2  versione con solo il sensore davanti, credo necessiti di un meccanismo di memoria
        """funzione per seguire il muro -> mi da la omega mentre sto navigando vicino al muro per evitare di allontanarmi dal muro
        Rifatta per usare le distanze laterali
        Sfrutta anche self.WF_ref_angle[i] angolo di partenza del WF
        Returns
        --------
        omega: tipo
            descrizione
        vel: tipo
            descrizione
        WFstate: int
            aggiorna il WFstate
        """
        outside_region_omega_reduction_factor = 1 # buono:1.5 ## prova 2
        inside_region_omega_reduction_factor = 0.3 # buono:0.4 ## TODO: aggiusta per far sì che raggiunga un'andamento bello parallelo al muro ASAP
        
        alfa = self.rpy[nth_drone][2] - self.WF_ref_angle[nth_drone]  # pos se sbando verso sinsitra

        if self.S_WF == 1: # wallfollowing con muro a destra
            lat_distance = self.observation[nth_drone][3] # distanza destra
            lat_distance = lat_distance * np.cos(alfa)
            prev_lat_distance = self.prev_rR[nth_drone][0]
        elif self.S_WF == -1: # wallfollowing con muro a sinistra
            lat_distance = self.observation[nth_drone][1] # distanza sinistra
            lat_distance = lat_distance * np.cos(alfa)
            prev_lat_distance = self.prev_rL[nth_drone][0]
            
        if np.abs(self.DIST_WALL_REF-lat_distance) > self.td : # sono fuori dalla regione accettabile ( o troppo lontano (distance > dist_wall_ref)) o troppo vicino  (distance < dist_wall_ref)
            if lat_distance > self.DIST_WALL_REF : #and self.rpy[nth_drone][2] < self.WF_ref_angle[nth_drone]:     # troppo lontano dal muro
                omega = ([-self.S_WF[nth_drone][0]*self.C_OMEGA*outside_region_omega_reduction_factor])        # TODO check sign : CHECKED -
                print("sono fuori dalla regione e troppo lontano dal muro")
            elif lat_distance < self.DIST_WALL_REF:                                                # troppo vicino al muro
                omega = ([+self.S_WF[nth_drone][0]*self.C_OMEGA*outside_region_omega_reduction_factor])  # +
                print("sono fuori dalla regione e troppo vicino al muro")       
        elif np.abs(self.DIST_WALL_REF-lat_distance) <  self.td : #sono dentro alla regione accettabile
            # meccanismo di fine tune alignment basato su confronto con distance[iesimo drone][0]
            if prev_lat_distance > lat_distance: # se la distanza dal muro prima era maggiore (mi sto avvicinando)
                omega = ([+self.S_WF[nth_drone][0]*self.C_OMEGA*inside_region_omega_reduction_factor])     # TODO andiamo a culo vedi se cambiare
                #omega = ([0.])
                print("sono dentro alla regione e mi sto avvicinando al muro")
            else : 
                omega = ([-self.S_WF[nth_drone][0]*self.C_OMEGA*inside_region_omega_reduction_factor])
                #omega = ([0.])
                print("sono dentro alla regione e mi sto allontanando dal muro")
              
        else :
            omega = ([0.])
        return omega      
    
    ################################################################################

    def _MinDistToWall2(self):
        """Distanza minima dall'ostacolo.
            ATTENZIONE: nelle fasi di volo circa parallelo al muro non ritorna la distanza minima esatta ma l'observation del RF laterale
        TODO richiede sWF per capire da che lato  (e faccio che la aggiorna???)

        Updates
        ----------
        self.distance : ndarray   [[prev_dist , curr_dist]]
            (NUM_DRONES, 2)-shaped array containing the minimal distance observed  , se vale self.MAX_RANGE vuol dire che non vede proprio niente
        self.S_WF
            (NUM_DRONES, 1)-shaped array 
        """
        for i in range(self.NUM_DRONES):
            rF = self.observation[i][0] # distanza frontale
            rL = self.observation[i][1] # distanza sinistra
            rB = self.observation[i][2] # distanza retro
            rR = self.observation[i][3] # distanza destra
            if rF != self.MAX_RANGE and rL != self.MAX_RANGE :
                NWalfa = np.arctan(rF/rL) #complementare dell'angolo di "raddrizzamento" rispetto al muro
                NWdistance = rF * np.cos(NWalfa)
            else:
                NWdistance = np.inf

            if rL != self.MAX_RANGE and rB != self.MAX_RANGE :
                SWalfa = np.arctan(rB/rL) #complementare dell'angolo di "raddrizzamento" rispetto al muro
                SWdistance = rB * np.cos(SWalfa)
            else:
                SWdistance = np.inf

            if rB != self.MAX_RANGE and rR != self.MAX_RANGE :
                SEalfa = np.arctan(rB/rR) #complementare dell'angolo di "raddrizzamento" rispetto al muro
                SEdistance = rB * np.cos(SEalfa)
            else:
                SEdistance = np.inf

            if rR != self.MAX_RANGE and rF != self.MAX_RANGE :
                NEalfa = np.arctan(rF/rR) #complementare dell'angolo di "raddrizzamento" rispetto al muro
                NEdistance = rF * np.cos(NEalfa)
            else:
                NEdistance = np.inf
            self.distance[i][1] = min(rF,rL,rB,rR,NWdistance,SWdistance,SEdistance,NEdistance)
            ######### spostato nel momento in cui si esce da WFSTATE = 0 #########
            # Imposta sWF in base al valore minimo
            #if self.distance[i][1] in [rL, NWdistance, SWdistance]:
            #    self.S_WF[i] = -1
            #elif self.distance[i][1] in [rR, NEdistance, SEdistance]:
            #    self.S_WF[i] = 1
            #else:
            #    self.S_WF[i] = 1  # preferenza antioraria
        return NWdistance,SWdistance,SEdistance,NEdistance
    
############################## CREAZIONE MAPPA TOPOLOGICA ############################################
## il database si chiama 
## self.drones_db
## la matrice di adiacenza è una sola
## self.adjacency_matrix

    def add_drone(self,
                  drone_id):
        """Aggiunge un drone col suo id nel database come chiave di primo livello drone_id
           e inizializza la matrice di adiacenza aggiungendo una matrice (1x1) con uno zero alla lista di matrici nota come self.adjacency_matrices
           Runnare in fase di inizializzazione
        Parameters
        -------
        drone_id: int
            id del drone da aggiungere
        """
        if drone_id not in self.drones_db:
            self.drones_db[drone_id] = {}
            adjacency_matrix = np.zeros((1, 1), dtype=int)
            self.adjacency_matrices.append(adjacency_matrix)

    def get_next_point_id(self,
                          drone_id):
        """restituisce il prossimo point_id inseribile nel dizionario del drone con un certo drone_id
        Parameters
        -------
        drone_id: int
            id del drone di cui voglio sapere quale sarebbe il prossimo point_id

        Returns
        -------
        string : 'xxxx'
            stringa col prossimo point_id
        """
        if drone_id not in self.drones_db:
            return '0001'
        else:
            existing_ids = sorted(int(pid) for pid in self.drones_db[drone_id].keys())
            next_id = existing_ids[-1] + 1 if existing_ids else 1
            return f'{next_id:04}'
        
    def add_point(self,
                  drone_id,
                  coords,
                  point_type):
        """aggiunge un punto al database del drone selezionato
        Parameters
        -------
        drone_id : int
            id del drone alla cui lista voglio aggiungere un punto
        coords : ndarray
            (3)-shaped array con le coord del punto
        point_type : string
            può essere 'corridor', 'juction', 'corner'          
        """
        new_point_threshold_distance = 0.7 #TODO: tuning
        if drone_id not in self.drones_db:
            self.drones_db[drone_id] = {}
        ### TODO Aggiungere logica di scelta se mettere o no in base 
        min_dist = self.distance_between_newpoint_and_oldpoints(drone_id,coords)
        if min_dist > new_point_threshold_distance or point_type == 'junction':
            ### aggiunta nuova chiave al dizionario
            point_id = self.get_next_point_id(drone_id)
            self.drones_db[drone_id][point_id] = {'coords': coords.copy(), 'type': point_type}
            print(f'Added point {point_id} to drone {drone_id}')
            ### aggiunta nuova riga e colonna alla matrice[drone_id]
            current_matrix = self.adjacency_matrices[drone_id]
            num_nodes = len(self.drones_db[drone_id])
            if num_nodes > 1:
                # Creiamo una nuova matrice con una dimensione aumentata di 1
                new_matrix = np.zeros((num_nodes , num_nodes ), dtype=int)
                # Copiamo i valori della matrice corrente nella nuova matrice
                new_matrix[:num_nodes-1, :num_nodes-1] = current_matrix
                # Aggiorniamo la lista delle matrici di adiacenza
                self.adjacency_matrices[drone_id] = new_matrix
            if point_type == 'corridor':
                color = [1, 0, 0, 1] # red
            elif point_type == 'junction':
                color = [0, 1, 0, 1] # green
            self.add_visual_ball(coords, color)

    def add_edge(self,
                 drone_id,
                 point_key1,
                 point_key2):
        """Da runnare dopo add_point
        scelto un drone drone_id, aggiunge il collegamento (edge) tra un nodo 1 e un nodo 2 
        Parameters
        -------
        drone_id: int
            id del drone in questione
        point_key1: str
            key del nodo 1 
        point_key2: str
            key del nodo 2
        """
        keys = list(self.drones_db[drone_id].keys())
        idx1 = keys.index(point_key1)
        idx2 = keys.index(point_key2)
        self.adjacency_matrices[drone_id][idx1, idx2] = 1
        self.adjacency_matrices[drone_id][idx2, idx1] = 1

    def euclidean_distance(self,
                           point1,
                           point2):
        """Distanza euclidea tra due nparray di qualsiasi dimensione
        Parameters
        -------
        point1: ndarray 
        point2: ndarray 

        Returns
        --------
        float
            euclidean distance between 2 points
        """
        return np.linalg.norm(np.array(point1) - np.array(point2))

    def merge_similar_points_2_drones(self,
                                      drone1_id,
                                      drone2_id,
                                      threshold=0.5):
        """ confronto incrociato tra i nodi di 2 droni diversi e sostituisce con la media #TODO: aggiusta sensibilità
        Parameters
        -------
        drone1_id: int 
        drone2_id: int
        threshold: float - optional
        """
        drone1_points = self.drones_db[drone1_id]
        drone2_points = self.drones_db[drone2_id]
        for id1, data1 in drone1_points.items():
            for id2, data2 in drone2_points.items():
                if self.euclidean_distance(data1['coords'], data2['coords']) < threshold:
                    new_coords = np.mean([data1['coords'], data2['coords']], axis=0).tolist()
                    self.drones_db[drone1_id][id1]['coords'] = new_coords
                    self.drones_db[drone2_id][id2]['coords'] = new_coords

    def distance_between_newpoint_and_oldpoints(self,
                                                drone_id,
                                                coords,
                                                ):
        """ da runnare in add_point. distanza minima tra l'eventuale nuovo nodo e nodo più vicino già nel dizionario#TODO: aggiusta sensibilità
        Parameters
        -------
        drone_id: int 
            id del drone alla cui lista voglio aggiungere un punto
        coords : ndarray
            (3)-shaped array con le coord del punto
        
        Returns
        -------
        float
            distanza minima
        """
        drone_points = self.drones_db[drone_id]
        min_dist = 5.0
        for id, data in drone_points.items():
            if self.euclidean_distance(data['coords'], coords) < min_dist:
                min_dist = self.euclidean_distance(data['coords'], coords)
        return min_dist


    def remove_point(self,
                     drone_id,
                     point_id):
        """rimuove un punto al database del drone selezionato
        Parameters
        -------
        drone_id : int
            id del drone dalla cui lista voglio rimuovere un punto
        point_id : str  
            'xxxx' string 
        """
        if drone_id in self.drones_db and point_id in self.drones_db[drone_id]:
            del self.drones_db[drone_id][point_id]
            print(f'Removed point {point_id} from drone {drone_id}')
        else:
            print(f'Point {point_id} not found for drone {drone_id}')

    def display_drones_db(self):
        """mostra l'intero database
        """
        for drone_id, points in self.drones_db.items():
            print(f"Drone {drone_id}:")
            for point_id, details in points.items():
                print(f"  Point {point_id}: {details}")

    def add_visual_ball(self,
                        position,
                        color=[1, 0, 0, 1],
                        radius=0.03,
                        ):
        """crea un pallino colorato senza collisioni in un punto
        --------
        Parametres
        position: ndarray (3) 
            position of the pallino
        color: ndarray (4) 
            RGBA-format color
        radius: float - optional
        """
        visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=radius, rgbaColor=color)
        ball_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape_id, basePosition=position)
        return ball_id
    
    def add_visual_line(self,
                        point_a,
                        point_b,
                        color=[0, 0, 1, 1],
                        width=1):
        """crea una linea colorata senza collisioni da un punto a a un punto b
        --------
        Parametres
        point_a: ndarray (3) 
            posizione di partenza
        point_b: ndarray (3) 
            posizione di fine 
        color: ndarray (3) 
            RGB-format color
        width: float - optional
        """
        line_id = p.addUserDebugLine(point_a, point_b, color, width)
        return line_id