import numpy as np
import time

# Inizializza le variabili di odometria
position = np.array(start_position)
orientation = np.array([0, 0, 0])  # Roll, Pitch, Yaw

# Tempo di simulazione
time_step = 1/240

# Funzione per aggiornare l'odometria
def update_odometry(position, orientation, linear_velocity, angular_velocity, dt):
    # Aggiorna la posizione
    position += np.array(linear_velocity) * dt
    
    # Aggiorna l'orientamento (integrazione di Euler semplice)
    orientation += np.array(angular_velocity) * dt
    
    return position, orientation

# Inizializza gli errori
position_error = np.array([0.1, 0.1, 0.1])  # Stima dell'errore iniziale (in metri)

while True:
    p.stepSimulation()
    
    # Ottieni le velocità del drone
    linear_velocity, angular_velocity = p.getBaseVelocity(drone_id)
    
    # Converti le velocità da tuple a numpy arrays
    linear_velocity = np.array(linear_velocity)
    angular_velocity = np.array(angular_velocity)
    
    # Aggiorna l'odometria
    position, orientation = update_odometry(position, orientation, linear_velocity, angular_velocity, time_step)
    
    # Stima dell'errore accumulato (ipotizziamo un errore incrementale)
    position_error += np.array([0.01, 0.01, 0.01])  # Aumenta l'errore stimato ad ogni passo
    
    # Stampa la posizione e l'errore
    print(f"Position: {position}, Orientation: {orientation}, Error: {position_error}")
    
    # Aspetta per rispettare il passo di tempo della simulazione
    time.sleep(time_step)