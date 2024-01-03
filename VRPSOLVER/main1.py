#import libraries
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from generate_data import generate_data
from ortools.linear_solver import pywraplp



generate_data(5)


def read_excel_file(filename, sheet_name):
    """
    Read coordinates and demand values from a specific sheet in an Excel file.
    Assumes the data is in columns labeled 'X', 'Y', and 'Demand'.
    """
    df = pd.read_excel(filename, sheet_name=sheet_name)
    nodes = df[['X', 'Y']].values
    demands = df['Demand'].values
    return nodes, demands

class Node:
    def __init__(self, index, x, y, demand):
        self.index = index
        self.x = x
        self.y = y
        self.demand = demand

class Vehicle:
    def __init__(self, capacity):
        self.capacity = capacity
        self.current_load = 0

    def can_accommodate(self, demand):
        return self.current_load + demand <= self.capacity




def calculate_distance(point1, point2):
    """
    Calculate the Euclidean distance between two points.
    """
    x1, y1 = point1
    x2, y2 = point2
    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def calculate_distance_matrix(coordinates):
    """
    Calculate the distance matrix between coordinates.
    """
    num_points = len(coordinates)
    dist_matrix = np.zeros((num_points, num_points))

    for i in range(num_points):
        for j in range(num_points):
            dist_matrix[i, j] = calculate_distance(coordinates[i], coordinates[j])

    return dist_matrix




def calculate_total_distance(route, dist_matrix):
    """
    Calculate the total distance of a given route using the distance matrix.
    """
    total_distance = 0
    num_points = len(route)

    for i in range(num_points - 1):
        current_node = route[i]
        next_node = route[i + 1]
        total_distance += dist_matrix[current_node, next_node]

    return total_distance


def nearest_neighbor(dist_matrix, demands, capacity):
    """
    Apply the Nearest Neighbor heuristic to find initial routes for VRP.
    """
    num_points = dist_matrix.shape[0]
    visited = np.zeros(num_points, dtype=bool)
    routes = []

    while np.sum(visited) < num_points:
        current_node = 0  # Start at node 0
        current_capacity = 0
        route = [current_node]
        visited[current_node] = True

        while current_capacity + demands[current_node] <= capacity:
            current = route[-1]
            nearest = None
            min_dist = float('inf')

            for neighbor in np.where(~visited)[0]:
                if demands[neighbor] + current_capacity <= capacity and dist_matrix[current, neighbor] < min_dist:
                    nearest = neighbor
                    min_dist = dist_matrix[current, neighbor]

            if nearest is None:
                break

            route.append(nearest)
            visited[nearest] = True
            current_capacity += demands[nearest]

        routes.append(route)

    return routes





def vrp_solver(filename, sheet_name, capacity):
    """
    Solve the VRP using the provided filename for coordinates and vehicle capacity.
    """
    coordinates, demands = read_excel_file(filename, sheet_name)
    nodes = [Node(index, coord[0], coord[1], demand) for index, (coord, demand) in enumerate(zip(coordinates, demands))]
    dist_matrix = calculate_distance_matrix(coordinates)
    routes = nearest_neighbor(dist_matrix, demands, capacity)
    formatted_routes = format_output(routes, nodes)
    return formatted_routes


def format_output(routes, nodes):
    """
    Format the final routes as required.
    In this example, it returns a list of routes.
    """
    formatted_routes = []
    for route in routes:
        formatted_route = [(nodes[i].x, nodes[i].y) for i in route]
        formatted_routes.append(formatted_route)
    return formatted_routes


# filename = r"C:\\Users\\KASSA\\Documents\\scientifique\\VRPSOLVER\\data.xlsx"
# sheet_name = "Sheet1"
# capacity = 2010
def plot_routes(routes):
    """
    Plot the routes on a 2D graph.
    """
    for route in routes:
        x = [point[0] for point in route]
        y = [point[1] for point in route]
        plt.plot(x, y, marker='o', linestyle='-')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('VRP Solution')
    plt.show()



# Ensure demands is defined
# solution = vrp_solver(filename, sheet_name, capacity)
# print(solution)
# plot_routes(solution) 

def solve_vrp_ilp_with_constraints(coordinates, demands, capacity, time_windows=None):
    num_nodes = len(coordinates)
    solver = pywraplp.Solver.CreateSolver('SCIP')

    # Variables de décision
    x = {}
    for i in range(num_nodes):
        for j in range(num_nodes):
            if i != j:
                x[i, j] = solver.BoolVar(f'x[{i},{j}]')

    u = {}
    for i in range(num_nodes):
        u[i] = solver.NumVar(0, solver.infinity(), f'u[{i}]')

    # Fonction Objectif
    objective = solver.Objective()
    for i in range(num_nodes):
        for j in range(num_nodes):
            if i != j:
                objective.SetCoefficient(x[i, j], calculate_distance(coordinates[i], coordinates[j]))

    objective.SetMinimization()

    # Contraintes
    for i in range(num_nodes):
        solver.Add(solver.Sum([x[i, j] for j in range(num_nodes) if i != j]) == 1)

    for j in range(num_nodes):
        solver.Add(solver.Sum([x[i, j] for i in range(num_nodes) if i != j]) == 1)

    for i in range(num_nodes):
        for j in range(num_nodes):
            if i != j:
                solver.Add(u[i] + demands[j] - u[j] <= capacity * (1 - x[i, j]))
    for i in range(num_nodes):
        solver.Add(solver.Sum([x[i, j] for j in range(num_nodes) if i != j]) == 1)

    for j in range(num_nodes):
        solver.Add(solver.Sum([x[i, j] for i in range(num_nodes) if i != j]) == 1)

    for i in range(num_nodes):
        for j in range(num_nodes):
            if i != j:
                solver.Add(u[i] + demands[j] - u[j] <= capacity * (1 - x[i, j]))

    # Nouvelles contraintes pour le VRP
    for i in range(1, num_nodes):  # Ignorer le dépôt
        for j in range(1, num_nodes):
            if i != j:
                solver.Add(u[i] + demands[i] <= u[j] + capacity * (1 - x[i, j]))
    # Contraintes de fenêtre temporelle
    if time_windows:
        for i in range(num_nodes):
            if i < len(time_windows) and time_windows[i]:
                solver.Add(u[i] >= time_windows[i][0])
                solver.Add(u[i] <= time_windows[i][1])

    # Contraintes de sous-tour
    subtour_elimination(solver, x, num_nodes)

    # Résoudre le problème
    status = solver.Solve()

    if status == pywraplp.Solver.OPTIMAL or status == pywraplp.Solver.FEASIBLE:
        print('Solution trouvée.')

        # Afficher les variables de décision (chemins sélectionnés)
        print('Chemins sélectionnés:')
        for i in range(num_nodes):
            for j in range(num_nodes):
                if i != j and x[i, j].solution_value() > 0.5:  # Chemin sélectionné
                    print(f'Chemin de {i} à {j}')

        # Autres traitements de la solution si nécessaire

    else:
        print('Aucune solution trouvée.')

def subtour_elimination(solver, x, num_nodes):
    """
    Ajoute des contraintes de sous-tour pour éliminer les cycles non désirés.
    """
    visited = np.zeros(num_nodes, dtype=bool)

    while True:
        current_node = np.argmax(visited == 0)
        if current_node == 0:  # Dépôt
            break

        subtour = [current_node]
        visited[current_node] = True

        while True:
            current_node = np.argmax([x[subtour[-1], j].solution_value() for j in range(num_nodes) if not visited[j]])
            if current_node == 0:  # Dépôt
                break
            subtour.append(current_node)
            visited[current_node] = True

        if len(subtour) < num_nodes:
            solver.Add(solver.Sum([x[i, j] for i in subtour for j in subtour if i != j]) >= 2)

filename = r"C:\\Users\\KASSA\\Documents\\scientifique\\VRPSOLVER\\data.xlsx"
sheet_name = "Sheet1"
capacity = 2500
time_windows = [(0, 20), (0, 20), (0, 20), (0, 20), (0, 20), (0, 20)]
# Après la résolution du VRP par l'approche du plus proche voisin
solution = vrp_solver(filename, sheet_name, capacity)

# Récupérer les coordonnées et les demandes du VRP résolu
coordinates, demands = read_excel_file(filename, sheet_name)

# Appeler la fonction ILP avec contraintes
solve_vrp_ilp_with_constraints(coordinates, demands, capacity, time_windows)