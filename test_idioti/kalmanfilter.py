# PSEUDO CODICE PER UN KALMAN FILTER

# Inizializza le matrici del filtro di Kalman
state = np.zeros(6)  # [x, y, z, roll, pitch, yaw]
P = np.eye(6)  # Covarianza dell'errore
Q = np.eye(6) * 0.1  # Rumore del processo
R = np.eye(6) * 0.1  # Rumore delle misure

def kalman_filter_update(state, P, z, u, dt):
    # Predizione dello stato
    F = np.eye(6) + dt * np.block([[np.zeros((3, 3)), np.eye(3)], [np.zeros((3, 6))]])
    state_pred = F @ state + dt * u
    P_pred = F @ P @ F.T + Q
    
    # Aggiornamento con le misure
    H = np.eye(6)  # Matrice di osservazione
    K = P_pred @ H.T @ np.linalg.inv(H @ P_pred @ H.T + R)
    state = state_pred + K @ (z - H @ state_pred)
    P = (np.eye(len(K)) - K @ H) @ P_pred
    
    return state, P

# Simulazione
while True:
    p.stepSimulation()
    
    # Ottieni le velocità del drone
    linear_velocity, angular_velocity = p.getBaseVelocity(drone_id)
    u = np.hstack((linear_velocity, angular_velocity))  # Controllo del movimento
    
    # Aggiorna l'odometria con rumore
    position, orientation = update_odometry_with_noise(position, orientation, linear_velocity, angular_velocity, time_step, 0.01, 0.01)
    
    # Misura (aggiungi rumore alle misure per simulare sensori)
    z = np.hstack((position, orientation)) + np.random.normal(0, 0.1, 6)
    
    # Aggiorna il filtro di Kalman
    state, P = kalman_filter_update(state, P, z, u, time_step)
    
    # Stampa la posizione stimata e l'errore
    print(f"Position: {state[:3]}, Orientation: {state[3:]}")
    
    time.sleep(time_step)