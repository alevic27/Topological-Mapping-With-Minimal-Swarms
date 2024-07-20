import numpy as np

# Nuovi dati
nuovi_dati = [76.69 ,
85.83 ,
76.10 ,
90.23 ,
61.62 ,
109.71,
136.71,
188.12,
]

# Calcolo della media per i nuovi dati
nuova_media = np.mean(nuovi_dati)

# Calcolo della deviazione standard per i nuovi dati
nuova_deviazione_standard = np.std(nuovi_dati, ddof=1)

print(nuova_media)
print( nuova_deviazione_standard)