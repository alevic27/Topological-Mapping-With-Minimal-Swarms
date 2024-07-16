import heapq
import numpy as np

class DroneNavigation:
    def __init__(self, drones_db, adjacency_matrices,pos):
        """
        Initialize the DroneNavigation class.
        
        Parameters:
        ----------
        drones_db : dict
            A dictionary containing the nodes and their data for each drone.
        adjacency_matrices : dict
            A dictionary containing adjacency matrices for each drone.
        """
        self.drones_db = drones_db
        self.adjacency_matrices = adjacency_matrices
        self.pos = pos

    def point_id_to_index(self, point_id:str):
        """
        Converte una stringa di point_id del tipo 'xxxx' nel corrispondente indice Python (intero).
        Params:
        point_id : str
            Stringa del tipo 'xxxx'.

        Returns:
        int
            Indice Python corrispondente (0-based).
        """
        return int(point_id) - 1
    
    def index_to_point_id(self, index: int):
        """
        Converte un indice Python (intero) nel corrispondente stringa di point_id del tipo 'xxxx'.
        Params:
        index : int
            Indice Python (0-based).

        Returns:
        str
            Stringa del tipo 'xxxx' corrispondente.
        """
        return f'{index + 1:04}'

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

    def get_closest_node(self,
                         drone_id):
        """
        trova il nodo più vicino da cui iniziare il tracciato

        Parameters
        ----------
        drone_id : int
            id del drone

        Returns
        ----------
        str
            L'ID del punto più vicino da cui partire per il ritorno
        """
        actual_position = self.pos[drone_id]
        drone_points = self.drones_db[drone_id]
        min_dist = np.inf
        closest_point_id = []
        for point_id, data in drone_points.items():
            distance = self.euclidean_distance(actual_position,data['coords'])
            if distance < min_dist:
                min_dist = distance
                closest_point_id = point_id
        return closest_point_id

    def dijkstra(self,
                 drone_id,
                 start_node):
        """
        Implementa l'algoritmo di Dijkstra per trovare il percorso più breve da start_node a tutti gli altri nodi.

        Params:
        drone_id : int
            ID del drone.
        start_node : str
            ID del nodo di partenza 'start'

        Returns:
        dict
            Distanze minime dal nodi di partenza a tutti gli altri nodi.
        dict
            Nodo precedente nel percorso più breve per ciascun nodo.
        """
        nodes = list(self.drones_db[drone_id].keys())
        adjacency_matrix = self.adjacency_matrices[drone_id]
        num_nodes = len(nodes)

        visited = set()
        distances = {node: float('inf') for node in nodes}
        previous_nodes = {node: None for node in nodes}
        distances[start_node] = 0

        while len(visited) < num_nodes:
            current_node = min((node for node in nodes if node not in visited), key=lambda node: distances[node])
            visited.add(current_node)
            #current_index = self.point_id_to_index(current_node)

            for neighbor_index, weight in enumerate(adjacency_matrix[self.point_id_to_index(current_node)]):
                neighbor_node = self.index_to_point_id(neighbor_index)
                if weight > 0 and neighbor_node in nodes and neighbor_node not in visited:
                    new_distance = distances[current_node] + weight
                    if new_distance < distances[neighbor_node]:
                        distances[neighbor_node] = new_distance
                        previous_nodes[neighbor_node] = current_node

        return distances, previous_nodes

    def find_path_to_start(self,
                           drone_id):
        """
        Trova il percorso più breve dal nodo più vicino alla posizione corrente del drone al nodo di partenza.

        Params:
        drone_id : int
            ID del drone.

        Returns:
        list
            Lista ordinata di nodi da raggiungere per tornare al nodo di partenza.
        """
        # Trova il nodo più vicino alla posizione corrente da cui partire
        closest_node_id = self.get_closest_node(drone_id)

        # Prende il nodo con il tag 'start'
        start_node_id = None
        for node_id, data in self.drones_db[drone_id].items():
            if data['type'] == 'start':
                start_node_id = node_id
                break
        if start_node_id is None:
            raise ValueError(f"Start node not found for drone {drone_id}")

        # Trova il path più veloce: closest node >>> start node
        distances, previous_nodes = self.dijkstra(drone_id, start_node_id)

        # Ricostruisce il percorso dal nodo più vicino al nodo di partenza
        path = []
        current_node = closest_node_id
        while current_node is not None:
            path.append(current_node)
            current_node = previous_nodes[current_node]

        return path

# Example usage
# Assuming self.drones_db and self.adjacency_matrices are already defined
drones_db = {
    0: {
        '0001' : {'coords': [-2.3       ,  0.        ,  0.99744854], 'type': 'start', 'addition': 'previous'},
        '0002' : {'coords': [-1.4292195786376658, 0.022519763433567742, 1.0098575747957583], 'type': 'junction', 'addition': 'previous'},
        '0003' : {'coords': [0.10761251292975722, 0.832806974640565, 1.0183548517292362], 'type': 'junction', 'addition': 'previous'},
        '0005' : {'coords': [0.5485982027653571, 4.170251237669295, 1.0515702317315674], 'type': 'corridor', 'addition': 'previous'},
        '0006' : {'coords': [-0.23771900466745266, 1.7917685981584477, 1.0279823937014205], 'type': 'corridor', 'addition': 'previous'},
        '0007' : {'coords': [-0.18482302687225086, 2.827720604035303, 1.0383707701781797], 'type': 'corridor', 'addition': 'previous'},
        '0008' : {'coords': [2.201565074435422, 3.936766618618564, 1.0756826610436059], 'type': 'corridor', 'addition': 'last'},
}}

adjacency_matrices = [
    [
        [0, 1, 0, 0, 0, 0, 0, 0],
        [1, 0, 1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 1, 0, 1],
        [0, 0, 0, 0, 0, 0, 1, 0]
    ],
    [
        [0,1,0],
        [1,0,1],
        [0,1,0],
    ]
]
pos = [[-0.2, 2., 1.], [0., 0., 0.]]



navigation = DroneNavigation(drones_db, adjacency_matrices, pos)
path = navigation.find_path_to_start(0)
print(f"Path from the closest node to the current position to the 'start' node: {path}")
