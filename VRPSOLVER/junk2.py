from ortools.linear_solver import pywraplp
import numpy as np

def create_data_model(coordinates, demands, vehicle_capacity, time_windows):
    """Create the data for the example."""
    data = {}
    data['distance_matrix'] = [
        [np.linalg.norm(np.array(coordinates[i])-np.array(coordinates[j]))
         if i != j else 0 for j in range(len(coordinates))]
        for i in range(len(coordinates))
    ]
    data['demands'] = demands
    data['vehicle_capacities'] = [vehicle_capacity]
    data['time_windows'] = time_windows
    data['num_vehicles'] = 1
    return data

def solve_vrp(data):
    """Solves the VRP with time windows."""
    # Instantiate the solver
    solver = pywraplp.Solver.CreateSolver('SCIP')
    # Solver parameters
    time_horizon = sum(data['time_windows'][i][1] for i in range(1, len(data['time_windows'])))

    # Define decision variables and other necessary variables
    x = {}
    for i in range(len(data['distance_matrix'])):
        for j in range(len(data['distance_matrix'])):
            if i != j:
                x[i, j] = solver.BoolVar('x[%i,%i]' % (i, j))
    u = {}
    for i in range(len(data['distance_matrix'])):
        u[i] = solver.NumVar(0, time_horizon, 'u[%i]' % i)

    # Define the objective: minimize total distance traveled
    objective = solver.Objective()
    for i in range(len(data['distance_matrix'])):
        for j in range(len(data['distance_matrix'][i])):
            if i != j:
                objective.SetCoefficient(x[i, j], data['distance_matrix'][i][j])
    objective.SetMinimization()

    # Add constraints
    for i in range(len(data['distance_matrix'])):
        solver.Add(solver.Sum([x[i, j] for j in range(len(data['distance_matrix'])) if i != j]) == 1)
        solver.Add(solver.Sum([x[j, i] for j in range(len(data['distance_matrix'])) if i != j]) == 1)

    # Capacity constraints
    for i in range(len(data['distance_matrix'])):
        for j in range(len(data['distance_matrix'])):
            if i != j:
                solver.Add(u[i] + data['demands'][j] <= u[j] + data['vehicle_capacities'][0] * (1 - x[i, j]))

    # Time window constraints
    for i in range(len(data['time_windows'])):
        solver.Add(u[i] >= data['time_windows'][i][0])
        solver.Add(u[i] <= data['time_windows'][i][1])

    # Subtour Elimination
    for i in range(1, len(data['distance_matrix'])):
        for j in range(1, len(data['distance_matrix'])):
            if i != j:
                solver.Add(u[i] - u[j] + data['vehicle_capacities'][0] * x[i, j] <= data['vehicle_capacities'][0] - data['demands'][i])

    # Solve the problem
    status = solver.Solve()

    if status == pywraplp.Solver.OPTIMAL or status == pywraplp.Solver.FEASIBLE:
        print('Solution found!')
        print('Total distance: ', objective.Value())
        for i in range(len(data['distance_matrix'])):
            for j in range(len(data['distance_matrix'][i])):
                if i != j and x[i, j].solution_value() > 0:
                    print(f'From node {i} to {j} with demand {data["demands"][j]}')
    else:
        print('No solution found.')

def main():
    # Provide example coordinates, demands, and capacity
    example_coordinates = [(0, 0), (1, 0), (2, 0), (3, 0)]  # Example coordinates
    example_demands = [0, 1, 2, 3]  # Example demands
    vehicle_capacity = 5  # Example capacity
    example_time_windows = [(0, 10), (0, 10), (1, 10), (2, 10)]  # Example time windows

    # Create data
    data = create_data_model(example_coordinates, example_demands, vehicle_capacity, example_time_windows)

    # Solve the VRP problem
    solve_vrp(data)

if __name__ == '__main__':
    main()