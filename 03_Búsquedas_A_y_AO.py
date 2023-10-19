import heapq

def astar(graph, start, goal):
    open_list = [(0, start, [])]  # (f(n), nodo, camino)
    closed_set = set()

    while open_list:
        f, node, path = heapq.heappop(open_list)
        if node == goal:
            return path + [node]

        if node in closed_set:
            continue

        closed_set.add(node)

        for neighbor, cost in graph.get(node, []):
            if neighbor not in closed_set:
                g = f - h(node) + cost + h(neighbor)
                heapq.heappush(open_list, (g, neighbor, path + [node]))

    return []

def h(node):
    # Heurística simple: distancia euclidiana al nodo objetivo
    x1, y1 = coordinates[node]
    x2, y2 = coordinates[goal]
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

# Ejemplo de uso:
graph = {
    'A': [('B', 2), ('C', 3)],
    'B': [('A', 2), ('D', 4)],
    'C': [('A', 3), ('D', 1)],
    'D': [('B', 4), ('C', 1)]
}

coordinates = {
    'A': (0, 0),
    'B': (1, 0),
    'C': (0, 1),
    'D': (1, 1)
}

start = 'A'
goal = 'D'
path = astar(graph, start, goal)

if path:
    print("Camino más corto:", path)
    print("Costo total:", sum(graph[node][0][1] for node in path[:-1]))
else:
    print("No se encontró un camino.")
