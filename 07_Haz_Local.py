import random
import numpy as np

# Ejemplo de un grafo ponderado (matriz de adyacencia)
graph = np.array([
    [0, 29, 20, 21, 16],
    [29, 0, 15, 17, 28],
    [20, 15, 0, 28, 23],
    [21, 17, 28, 0, 18],
    [16, 28, 23, 18, 0]
])

# Función objetivo: Costo total del ciclo
def objective_function(solution, graph):
    cost = 0
    n = len(solution)
    for i in range(n):
        cost += graph[solution[i]][solution[(i + 1) % n]]
    return cost

# Búsqueda local con reinicios aleatorios
def local_search(graph, max_iterations):
    n = len(graph)
    best_solution = list(range(n))
    best_cost = objective_function(best_solution, graph)

    for _ in range(max_iterations):
        current_solution = random.sample(range(n), n)
        current_cost = objective_function(current_solution, graph)

        while True:
            neighbors = []
            for i in range(n):
                for j in range(i + 1, n):
                    neighbor_solution = current_solution.copy()
                    neighbor_solution[i], neighbor_solution[j] = neighbor_solution[j], neighbor_solution[i]
                    neighbor_cost = objective_function(neighbor_solution, graph)
                    neighbors.append((neighbor_cost, neighbor_solution))

            neighbors.sort()
            if neighbors[0][0] >= current_cost:
                break

            current_solution = neighbors[0][1]
            current_cost = neighbors[0][0]

        if current_cost < best_cost:
            best_solution = current_solution
            best_cost = current_cost

    return best_solution, best_cost

# Parámetros de la búsqueda local
max_iterations = 1000

# Ejecución de la búsqueda local
best_solution, best_cost = local_search(graph, max_iterations)

# Resultados
print("Mejor solución encontrada:", best_solution)
print("Costo de la mejor solución:", best_cost)
