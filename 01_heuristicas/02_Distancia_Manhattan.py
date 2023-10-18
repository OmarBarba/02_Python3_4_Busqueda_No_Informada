import heapq

def manhattan_distance(node, goal):
    # Heurística: Distancia Manhattan desde 'node' hasta 'goal'
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

def a_star_search(graph, start, goal):
    open_list = []  # Cola de prioridad para nodos abiertos
    closed_set = set()  # Conjunto de nodos cerrados
    came_from = {}  # Diccionario para rastrear el camino desde el nodo inicial
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0

    # Inicializar la cola de prioridad con el nodo de inicio y su costo estimado
    heapq.heappush(open_list, (0 + manhattan_distance(start, goal), start))

    while open_list:
        _, current = heapq.heappop(open_list)  # Obtiene el nodo con el menor costo estimado desde la cola

        if current == goal:
            # Reconstruir el camino desde el objetivo hasta el inicio
            path = []
            while current in came_from:
                path.insert(0, current)
                current = came_from[current]
            path.insert(0, start)
            return path

        closed_set.add(current)

        for neighbor in graph[current]:
            tentative_g_score = g_score[current] + graph[current][neighbor]
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score = g_score[neighbor] + manhattan_distance(neighbor, goal)
                if neighbor not in [node[1] for node in open_list]:
                    heapq.heappush(open_list, (f_score, neighbor))

    return None  # No se encontró un camino

# Ejemplo de uso: Definición del grafo y nodos de inicio y objetivo
graph = {
    (0, 0): {(0, 1): 1, (1, 0): 1},
    (0, 1): {(0, 0): 1, (1, 1): 1},
    (1, 0): {(0, 0): 1, (1, 1): 1},
    (1, 1): {(0, 1): 1, (1, 0): 1}
}

start_node = (0, 0)  # Nodo de inicio
goal_node = (1, 1)   # Nodo objetivo

result = a_star_search(graph, start_node, goal_node)  # Llamada a la función A* con heurística de distancia Manhattan

if result:
    print("Camino encontrado:")
    print(result)  # Imprimir la ruta encontrada
else:
    print("No se encontró un camino.")
