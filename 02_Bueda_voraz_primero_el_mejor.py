
import heapq

# Definición del grafo (diccionario de nodos y sus coordenadas)
graph = {
    'A': (1, 2),
    'B': (4, 3),
    'C': (3, 6),
    'D': (8, 1),
    'E': (9, 5),
    'F': (7, 7)
}

# Nodo de inicio y nodo objetivo
start_node = 'A'
goal_node = 'F'

# Función para calcular la distancia Euclidiana entre dos puntos
def euclidean_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

# Función de búsqueda voraz
def best_first_search(graph, start, goal):
    visited = set()
    priority_queue = [(euclidean_distance(graph[start], graph[goal]), start)]

    while priority_queue:
        priority, node = heapq.heappop(priority_queue)

        if node not in visited:
            visited.add(node)
            if node == goal:
                return visited

            neighbors = graph.keys() - visited
            for neighbor in neighbors:
                heapq.heappush(priority_queue, (euclidean_distance(graph[neighbor], graph[goal]), neighbor))

    return visited

# Realizar la búsqueda
visited_nodes = best_first_search(graph, start_node, goal_node)

print("Nodos visitados en el orden de búsqueda voraz:")
print(visited_nodes)
