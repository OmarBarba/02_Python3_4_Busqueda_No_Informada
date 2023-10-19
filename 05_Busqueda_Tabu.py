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

# Búsqueda Tabú
def tabu_search(graph, max_iterations, tabu_tenure):
    n = len(graph)
    current_solution = list(range(n))
    best_solution = current_solution.copy()
    tabu_list = [set() for _ in range(n)]
    current_cost = objective_function(current_solution, graph)
    best_cost = current_cost

    for _ in range(max_iterations):
        neighbors = []
        for i in range(n):
            for j in range(i + 1, n):
                if i not in tabu_list[j] and j not in tabu_list[i]:
                    neighbor_solution = current_solution.copy()
                    neighbor_solution[i], neighbor_solution[j] = neighbor_solution[j], neighbor_solution[i]
                    neighbor_cost = objective_function(neighbor_solution, graph)
                    neighbors.append((neighbor_cost, neighbor_solution, i, j))

        neighbors.sort()
        found = False
        for cost, neighbor_solution, i, j in neighbors:
            if neighbor_solution != best_solution:
                current_solution = neighbor_solution
                current_cost = cost
                tabu_list[i].add(j)
                tabu_list[j].add(i)
                found = True
                break

        if current_cost < best_cost:
            best_solution = current_solution.copy()
            best_cost = current_cost

        if not found:
            break

    return best_solution, best_cost

# Parámetros de la búsqueda Tabú
max_iterations = 1000
tabu_tenure = 5

# Ejecución de la búsqueda Tabú
best_solution, best_cost = tabu_search(graph, max_iterations, tabu_tenure)

# Resultados
print("Mejor solución encontrada:", best_solution)
print("Costo de la mejor solución:", best_cost)
