import numpy as np

class DroneData:
    def __init__(self):
        self.drones_db = {}
        self.adjacency_matrices = []

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
            può essere 'corridor', 'juction', 'corner' #TODO: creare una logica di determinazione         
        """
        if drone_id not in self.drones_db:
            self.drones_db[drone_id] = {}
        ### aggiunta nuova chiave al dizionario
        point_id = self.get_next_point_id(drone_id)
        self.drones_db[drone_id][point_id] = {'coords': coords, 'type': point_type}
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
        """
        return np.linalg.norm(np.array(point1) - np.array(point2))

    def merge_similar_points_2_drones(self,
                                      drone1_id,
                                      drone2_id,
                                      threshold=0.5):
        """ confronto incrociato tra i nodi di 2 droni diversi e sostituisce con la media #TODO: aggiusta sensibilità
        Parameters
        -------
        drone1_id: ndarray 
        drone2_id: ndarray
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

#####################################################################################################################
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
    
# Esempio di utilizzo
graph = DroneData()

# Aggiungere alcuni droni
for i in range(3) :
    graph.add_drone(i)

print(graph.drones_db)
print(graph.adjacency_matrices)
# Aggiungere punti ai droni
graph.add_point(0, [3.3, 2.1, 1.0], 'junction')
graph.add_point(0, [3.6, 2.2, 1.0], 'corridor')
graph.add_point(0, [4.0, 2.2, 1.0], 'corridor')

graph.add_point(1, [4.3, 2.4, 1.0], 'crossway')
graph.add_point(1, [3.7, 2.3, 1.0], 'corridor')
graph.add_point(1, [3.1, 2.5, 1.0], 'corridor')

graph.add_point(2, [4.3, 2.4, 1.0], 'crossway')
graph.add_point(2, [3.1, 2.3, 1.0], 'corridor')
graph.add_point(2, [3.1, 2.5, 1.0], 'corridor')
# Visualizzare i dati
print("Dati dopo l'aggiunta dei punti:")
graph.display_drones_db()
graph.drones_db[0]['0004'] = {'coords': [1., 1., 1.0], 'type': 'prova'}
# merging
# graph.display_drones_db()
# graph.merge_similar_points_2_drones(0,1)
# graph.display_drones_db()
graph.display_drones_db()

# aggiungere edge
graph.add_edge(1,'0001','0003')
print(graph.adjacency_matrices)

min_dist = graph.distance_between_newpoint_and_oldpoints(1, [3.2, 2.5, 1.0])
print(min_dist)