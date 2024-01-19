import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Définition des paramètres
n_nodes = 80  # Nombre de nœuds
n_depots = 8  # Nombre de dépôts
n_vehicles = 6  # Nombre de véhicules (réduit pour correspondre aux dépôts)
capacity = 100  # Capacité des véhicules
max_time = 350  # Durée maximale d'un trajet
max_customers = 8  # Nombre maximal de clients par dépôt
max_speed = 45

# Génération des données
nodes = np.random.uniform(0, 100, size=(n_nodes, 2))  # Coordonnées des nœuds
depots = np.random.choice(n_nodes, size=n_depots, replace=False)  # Index des dépôts
vehicles = np.random.choice(n_nodes, size=n_vehicles, replace=False)  # Index des véhicules



demand = np.random.randint(1, 25, size=n_nodes)  # Demande de chaque nœud
distances = np.zeros((n_nodes, n_nodes))  # Matrice des distances entre les nœuds
visited = np.zeros((n_vehicles, n_nodes), dtype=int)
print(depots)
print(vehicles)



# Calcul des distances entre les nœuds
for i in range(n_nodes):
    for j in range(n_nodes):
        distances[i, j] = np.sqrt((nodes[i, 0] - nodes[j, 0]) ** 2 + (nodes[i, 1] - nodes[j, 1]) ** 2)

# Définition de la fonction objectif
def objective_function(solution):
    total_distance = 0
    for i in range(n_vehicles):
        for j in range(max_customers):
            total_distance += distances[solution[i, j], solution[i, j + 1]]
    return total_distance

# Définition des contraintes
def constraints(solution):
    # Contrainte de capacité
    for i in range(n_vehicles):
        if np.sum(demand[solution[i, :]]) > capacity:
            return False
    # Contrainte de temps
    for i in range(n_vehicles):
        total_time = 0
        for j in range(max_customers):
            total_time += distances[solution[i, j], solution[i, j + 1]] / max_speed
        if total_time > max_time:
            return False
    
    # Contrainte de nœuds visités une seule fois par véhicule
    for j in range(n_nodes):
        if np.sum(visited[:, j]) != 1:
            return False
    return True

# Algorithme de résolution
solution = np.zeros((n_vehicles, max_customers + 1), dtype=int)  # Solution initiale
for i in range(n_vehicles):
    solution[i, 0] = depots[i]
    remaining_customers = np.setdiff1d(np.arange(n_nodes), depots[i])
    
    # Sélection aléatoire des clients
    selected_customers = np.random.choice(remaining_customers, size=max_customers, replace=False)
    
    # Mise à jour de la matrice binaire des visites
    visited[i, selected_customers] = 1
    
    solution[i, 1:] = selected_customers

# Itérations de l'algorithme
for it in range(100):
    # Sélection d'une solution aléatoire
    random_solution = np.random.permutation(solution)
    # Calcul de la distance de la solution aléatoire
    random_distance = objective_function(random_solution)
    
    # Vérification des contraintes (capacité, temps, visites uniques)
    if random_distance < objective_function(solution) and constraints(random_solution):
        solution = random_solution

# Affichage de la solution
print(solution)
print(objective_function(solution))

# Nombre de commandes, de dépôts et de livreurs
n_commandes = n_nodes - n_depots  # Nombre de commandes
n_depo = n_depots  # Nombre de dépôts
n_livreurs = n_vehicles  # Nombre de livreurs

# Carte des nœuds et des dépôts
plt.figure(figsize=(10, 8))
plt.scatter(nodes[:, 0], nodes[:, 1], c='b', label=f'Clients ({n_commandes})', marker='o', s=50)
plt.scatter(nodes[depots, 0], nodes[depots, 1], c='r', label=f'Dépôts ({n_depo})', marker='s', s=100)
plt.xlabel('Coordonnée X')
plt.ylabel('Coordonnée Y')
plt.title(f'Affectation avec une résolution vrp simple ({n_livreurs} livreurs)')
plt.legend()
plt.grid(True)

# Trajets des véhicules
for i in range(n_vehicles):
    vehicle_route = solution[i]
    x = nodes[vehicle_route, 0]
    y = nodes[vehicle_route, 1]
    plt.plot(x, y, linestyle='-', marker='o', markersize=8, label=f'Véhicule {i+1}')
plt.legend()

# # Capacité des véhicules
# plt.figure(figsize=(6, 6))
# demand_per_vehicle = np.array([np.sum(demand[solution[i, :]]) for i in range(n_vehicles)])
# plt.pie(demand_per_vehicle, labels=[f'Véhicule {i+1}' for i in range(n_vehicles)], autopct='%1.1f%%')
# plt.title('Capacité des véhicules')
# plt.axis('equal')

# # Temps de trajet
# total_time_per_vehicle = np.zeros(n_vehicles)
# for i in range(n_vehicles):
#     total_time_per_vehicle[i] = np.sum(distances[solution[i, :-1], solution[i, 1:]]) / max_speed
# plt.figure()
# plt.bar(range(n_vehicles), total_time_per_vehicle)
# plt.xlabel('Véhicule')
# plt.ylabel('Temps de trajet')
# plt.title('Temps de trajet par véhicule')

# Distance totale
total_distance = objective_function(solution)
print(f'Distance totale parcourue : {total_distance}')

# Affichage
plt.show()