import numpy as np
from ortools.linear_solver import pywraplp

# Example Data
coordinates = np.array([(0, 0), (1, 0), (2, 0), (3, 0)])
demands = np.array([0, 1, 2, 3])
capacity = 5
time_windows = [(0, 10), (0, 10), (1, 10), (2, 10)]

def solve_vrp_ilp_with_constraints(coordinates, demands, capacity, time_windows=None):
    # Number of nodes
    num_nodes = len(coordinates)
    # Create solver using SCIP backend
    solver = pywraplp.Solver.CreateSolver('SCIP')

    # Decision Variables
    x = {} # x[i, j] = 1 if the path from node i to node j is taken, 0 otherwise
    for i in range(num_nodes):
        for j in range(num_nodes):
            if i != j:
                x[i, j] = solver.BoolVar(f'x[{i},{j}]')

    u = {} # u[i] = the load of the vehicle when arriving at node i
    for i in range(num_nodes):
        u[i] = solver.NumVar(0, solver.infinity(), f'u[{i}]')

    # Objective Function: Minimize the total distance
    objective = solver.Objective()
    for i in range(num_nodes):
        for j in range(num_nodes):
            if i != j:
                objective.SetCoefficient(x[i, j], np.linalg.norm(np.array(coordinates[i]) - np.array(coordinates[j])))
    objective.SetMinimization()

    # Constraints
    # Each node must be entered and left exactly once
    for i in range(num_nodes):
        solver.Add(solver.Sum([x[i, j] for j in range(num_nodes) if i != j]) == 1)
        solver.Add(solver.Sum([x[j, i] for j in range(num_nodes) if i != j]) == 1)

    # Capacity constraints: The load of the vehicle plus the demand at the next node minus
    # the load of the vehicle at the next node must be less than or equal to the capacity of the vehicle
    for i in range(num_nodes):
        for j in range(num_nodes):
            if i != j:
                solver.Add(u[i] + demands[j] - u[j] <= capacity * (1 - x[i, j]))

    # Time window constraints, if provided
    if time_windows:
        for i, window in enumerate(time_windows):
            solver.Add(u[i] >= window[0])
            solver.Add(u[i] <= window[1])

    # Contraintes de sous-tour
    visited = [False] * len(coordinates)
    for count in range(1, len(coordinates)):
        if not visited[count]:
            subtour_elimination(solver, x, u, demands, capacity, count, visited)

    # Solve the model and print the result
    status = solver.Solve()

    if status in (pywraplp.Solver.OPTIMAL, pywraplp.Solver.FEASIBLE):
        print('Solution found!')
        print('Total distance:', objective.Value())
        for i in range(num_nodes):
            for j in range(num_nodes):
                if i != j and x[i, j].solution_value() > 0:
                    print(f'Path from {i} to {j} with demand: {demands[j]}')
    else:
        print('No solution found.')

def subtour_elimination(solver, x, u, demands, capacity, start_node, visited):
    visited[start_node] = True
    subtour_constraints = []

    for i in range(1, len(visited)):
        if not visited[i]:
            subtour_constraints.append(x[start_node, i] + x[i, start_node])

    if subtour_constraints:
        # Ensure that at least one edge from the current node is taken
        solver.Add(solver.Sum(subtour_constraints) >= 1)

        # Ensure that no more than one edge to the current node is taken
        for i in range(1, len(visited)):
            if not visited[i]:
                solver.Add(solver.Sum(x[i, j] for j in range(len(visited)) if i != j) <= 1)
                solver.Add(solver.Sum(x[j, i] for j in range(len(visited)) if i != j) <= 1)

# Call the function with coordinates, demands, capacity and optional time windows
solve_vrp_ilp_with_constraints(coordinates, demands, capacity, time_windows)