def point_id_to_index(point_id:str):
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

def index_to_point_id(index: int):
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
print(drones_db[0])
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
print(adjacency_matrices[0])
# funzione di ricerca restituisce
closest_node_id='0008'
start_node='0001'
drone_id = 0


####   dijkstra
nodes = list(drones_db[drone_id].keys())
adjacency_matrix = adjacency_matrices[drone_id]
print(nodes)
num_nodes = len(nodes)
print(num_nodes)

visited = set()
distances = {node: float('inf') for node in nodes}
previous_nodes = {node: None for node in nodes}
distances[start_node] = 0
print(distances)

while len(visited) < num_nodes:
    # Select the unvisited node with the smallest distance
    current_node = min((node for node in nodes if node not in visited), 
                       key=lambda node: distances[node])
    visited.add(current_node)
    current_index = point_id_to_index(current_node)

    for neighbor_index, weight in enumerate(adjacency_matrix[point_id_to_index(current_node)]):
        neighbor_node = index_to_point_id(neighbor_index)
        if weight > 0 and neighbor_node not in visited:
            new_distance = distances[current_node] + weight
            if new_distance < distances[neighbor_node]:
                distances[neighbor_node] = new_distance
                previous_nodes[neighbor_node] = current_node

print(distances)
print(previous_nodes)

#### find_path_to_start
closest_node_id='0008'
start_node='0001'
drone_id = 0
# Calcola le distanze e i nodi precedenti usando l'algoritmo di Dijkstra
adjacency_matrix = adjacency_matrices[drone_id]
# distances, previous_nodes from djikstra

path = []
current_node = closest_node_id
while current_node is not None:
    path.append(current_node)
    current_node = previous_nodes[current_node]
    
#path.reverse()
print("il percorso è : ",path)