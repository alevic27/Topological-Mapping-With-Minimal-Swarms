import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union
from shapely.affinity import rotate

class DroneMissionEvaluator:
    def __init__(self, total_area_polygon):
        """
        Inizializza il valutatore di missione.

        Parameters
        ----------
        total_area_polygon : Polygon
            Poligono che rappresenta l'area totale del labirinto.
        """
        self.total_area_polygon = total_area_polygon
        self.coverage_percent = 0
        self.drones_db = {}  # Assumendo che il database dei droni sia un dizionario

    def calculate_coverage(self, radius):
        """
        Calcola la percentuale di copertura dell'area da parte dei droni.

        Parameters
        ----------
        radius : float
            Il raggio degli intorni circolari intorno a ciascun nodo.

        Returns
        -------
        float
            La percentuale di copertura dell'area.
        """
        covered_area = 0
        buffers = []

        for drone_id, points in self.drones_db.items():
            for point_id, data in points.items():
                coords = data['coords']
                point = Point(coords[0], coords[1])
                buffer = point.buffer(radius).intersection(self.total_area_polygon)
                buffers.append(buffer)

        union_buffers = unary_union(buffers)
        covered_area = union_buffers.area
        coverage_percent = (covered_area / self.total_area_polygon.area) * 100
        return coverage_percent

    def plot_coverage(self, evaluator, radius):
        """
        Visualizza l'area coperta dai droni all'interno del labirinto.

        Parameters
        ----------
        evaluator : DroneMissionEvaluator
            Un'istanza della classe DroneMissionEvaluator contenente le informazioni sui droni e il poligono dell'area totale.
        radius : float
            Il raggio degli intorni circolari intorno a ciascun nodo.

        Returns
        -------
        None
        """
        fig, ax = plt.subplots()

        # Traccia l'area totale del labirinto
        rotated_total_area = rotate(evaluator.total_area_polygon, 90, origin=(0, 0))
        x, y = rotated_total_area.exterior.xy
        ax.plot(x, y, label='Total Area')

        for drone_id, points in evaluator.drones_db.items():
            for point_id, data in points.items():
                coords = data['coords']
                point = Point(coords[0], coords[1])
                buffer = point.buffer(radius).intersection(evaluator.total_area_polygon)
                rotated_buffer = rotate(buffer, 90, origin=(0, 0))
                if rotated_buffer.geom_type == 'Polygon':
                    x, y = rotated_buffer.exterior.xy
                    ax.plot(x, y, label=f'Buffer {point_id}')
                elif rotated_buffer.geom_type == 'MultiPolygon':
                    for poly in rotated_buffer:
                        x, y = poly.exterior.xy
                        ax.plot(x, y, label=f'Buffer {point_id}')
                else:
                    rotated_point = rotate(point, 90, origin=(0, 0))
                    ax.plot(rotated_point.x, rotated_point.y, 'o', label=f'Buffer {point_id}')

            # Imposta le etichette degli assi
        plt.xlabel('y')
        plt.ylabel('x')
        plt.title('Coverage Area')
        # Ottieni i limiti degli assi
        xlim = ax.get_xlim()
        # Imposta le etichette invertite per l'asse x
        xticks = np.linspace(xlim[0], xlim[1], num=5)
        ax.set_xticks(xticks)
        ax.set_xticklabels([f'{int(label):.0f}' for label in xticks[::-1]])
        #plt.legend()
        plt.show()


 #### Esempio di utilizzo

# Definisci l'area totale del labirinto come un poligono
total_area_polygon = Polygon([(3.0, 1.0), (1.0, 1.0), (1.0, 3.1), (2.9, 3.1), (2.9 , 4.9), (-2.9 , 4.9),
                              (-2.9 , 3.1), (-1, 3.1), (-1, -3.1),(-2.9, -3.1),(-2.9, -4.9),(2.9, -4.9),
                                (2.9 , -3.1),(1., -3.1),(1., -1.0),(3., -1.)])
new_coords = [(-x, -y) for x, y in total_area_polygon.exterior.coords]
total_area_polygon = Polygon(new_coords)
print("l'area totale è: ",total_area_polygon.area)
env = DroneMissionEvaluator(total_area_polygon)
env.drones_db = {
    1: {
        '0001': {'coords': [0, 0], 'type': 'corridor'},
        '0002': {'coords': [2.5, 4.0], 'type': 'corridor'},
        '0003': {'coords': [-2.5, 4.0], 'type': 'corridor'},
        '0004': {'coords': [0, 4.0], 'type': 'corridor'},
        
        # Aggiungi altri nodi se necessario
    },
    # Aggiungi altri droni se necessario
}
# TODO percentuali aggiustate, rotazione da fare
coverage = env.calculate_coverage(radius=0.75)
print(f"Coverage Percent: {coverage}%")
print(type(env))
env.plot_coverage(env, radius=0.75)