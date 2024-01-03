import pandas as pd
import numpy as np

def generate_data(N):
    # Définir le nombre de lignes (locations) N
    #N = 25

    # Générer des données aléatoires pour les colonnes
    locations = [f'{i+1}' for i in range(N)]
    x_coordinates = np.random.randint(-1000, 1000, N)
    y_coordinates = np.random.randint(-1000, 1000, N)
    Demand = np.random.randint(0,600,N)   # Valeurs aléatoires entre 0 et 600

    # Créer un DataFrame avec les données
    data = {'Location': locations, 'X': x_coordinates, 'Y': y_coordinates, 'Demand': Demand}
    df = pd.DataFrame(data)

    # Écrire le DataFrame dans un fichier Excel
    excel_file_path = 'data.xlsx'
    df.to_excel(excel_file_path, index=False)

    print(f'Fichier Excel généré avec succès : {excel_file_path}')
generate_data(25)