import random
import math

# Ejemplo de un grafo ponderado (matriz de adyacencia)
graph = [
    [0, 29, 20, 21, 16],
    [29, 0, 15, 17, 28],
    [20, 15, 0, 28, 23],
    [21, 17, 28, 0, 18],
    [16, 28, 23, 18, 0]
]

# Función objetivo: Costo total del ciclo
def objective_function(solution, graph):
    cost = 0
    n = len(solution)
    for i in range(n):
        cost += graph[solution[i]][solution[(i + 1) % n]]
    return cost

# Temple Simulado
def simulated_annealing(graph, max_iterations, initial_temperature, cooling_rate):
    n = len(graph)
    current_solution = list(range(n))
    best_solution = current_solution.copy()
    current_cost = objective_function(current_solution, graph)
    best_cost = current_cost
    temperature = initial_temperature

    for iteration in range(max_iterations):
        i, j = random.sample(range(n), 2)
        neighbor_solution = current_solution.copy()
        neighbor_solution[i], neighbor_solution[j] = neighbor_solution[j], neighbor_solution[i]
        neighbor_cost = objective_function(neighbor_solution, graph)

        delta = neighbor_cost - current_cost
        if delta < 0 or random.random() < math.exp(-delta / temperature):
            current_solution = neighbor_solution
            current_cost = neighbor_cost

        if current_cost < best_cost:
            best_solution = current_solution.copy()
            best_cost = current_cost

        temperature *= cooling_rate

    return best_solution, best_cost

# Parámetros de Temple Simulado
max_iterations = 10000
initial_temperature = 1000.0
cooling_rate = 0.995

# Ejecución de Temple Simulado
best_solution, best_cost = simulated_annealing(graph, max_iterations, initial_temperature, cooling_rate)

# Resultados
print("Mejor solución encontrada:", best_solution)
print("Costo de la mejor solución:", best_cost)
