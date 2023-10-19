#######################################
########Busqueda en tiempo real########
#######################################

import heapq

# Ejemplo de un grafo ponderado (matriz de adyacencia)
graph = {
    'A': {'B': 2, 'C': 5},
    'B': {'A': 2, 'D': 3, 'E': 1},
    'C': {'A': 5, 'F': 4},
    'D': {'B': 3},
    'E': {'B': 1, 'F': 6},
    'F': {'C': 4, 'E': 6}
}

# Función objetivo: Costo total del camino
def objective_function(path, graph):
    cost = 0
    for i in range(len(path) - 1):
        cost += graph[path[i]][path[i+1]]
    return cost

# Búsqueda en línea con A*
def online_search(graph, start, goal):
    visited = set()
    queue = [(0, [start])]

    while queue:
        cost, path = heapq.heappop(queue)
        node = path[-1]

        if node == goal:
            return path  # Devolver el camino completo si se alcanza el objetivo

        if node not in visited:
            visited.add(node)

            for neighbor, weight in graph[node].items():
                if neighbor not in visited:
                    new_cost = cost + weight
                    new_path = path + [neighbor]
                    heapq.heappush(queue, (new_cost, new_path))

# Ejemplo de uso: Búsqueda en línea desde 'A' a 'F'
start_node = 'A'
goal_node = 'F'
result = online_search(graph, start_node, goal_node)

if result:
    print("Mejor camino encontrado:", result)
    print("Costo del camino:", objective_function(result, graph))
else:
    print("No se encontró un camino.")
