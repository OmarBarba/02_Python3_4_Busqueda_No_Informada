import heapq

def minimum_spanning_tree_cost(graph):
    # Heurística: Costo mínimo del árbol de expansión en el grafo ponderado.
    # En este ejemplo, simplemente sumamos los costos de todas las aristas del grafo.
    total_cost = sum(weight for edges in graph.values() for weight in edges.values())
    return total_cost

def minimum_spanning_tree(graph):
    open_list = []  # Cola de prioridad para las aristas del grafo.
    closed_set = set()  # Conjunto de aristas visitadas.
    minimum_tree = {}  # Diccionario para el árbol de expansión de costo mínimo.

    # Inicializar la cola de prioridad con las aristas del grafo.
    for node, edges in graph.items():
        for neighbor, weight in edges.items():
            heapq.heappush(open_list, (weight, node, neighbor))

    while open_list:
        weight, node, neighbor = heapq.heappop(open_list)  # Obtiene la arista con el menor costo desde la cola.

        if node not in closed_set and neighbor not in closed_set:
            # Agregar la arista al árbol de expansión.
            if node not in minimum_tree:
                minimum_tree[node] = {}
            minimum_tree[node][neighbor] = weight
            if neighbor not in minimum_tree:
                minimum_tree[neighbor] = {}
            minimum_tree[neighbor][node] = weight
            closed_set.add(node)
            closed_set.add(neighbor)

    return minimum_tree

# Ejemplo de uso: Definición del grafo ponderado.
graph = {
    'A': {'B': 2, 'C': 5},
    'B': {'A': 2, 'D': 3, 'E': 1},
    'C': {'A': 5, 'F': 4},
    'D': {'B': 3},
    'E': {'B': 1, 'F': 6},
    'F': {'C': 4, 'E': 6}
}

minimum_tree = minimum_spanning_tree(graph)  # Llamada a la función para encontrar el árbol de expansión.

minimum_tree_cost = minimum_spanning_tree_cost(minimum_tree)  # Llamada a la función para calcular el costo del árbol.

print("Árbol de expansión de costo mínimo:")
print(minimum_tree)
print("Costo total del árbol de expansión de costo mínimo:", minimum_tree_cost)


#############################
########en la busqueda#######


def minimum_spanning_tree_cost(graph):
    # Heurística: Costo mínimo del árbol de expansión del grafo.
    # En este ejemplo, se utiliza el algoritmo de Kruskal para encontrar el costo mínimo del árbol de expansión.
    
    # Implementación del algoritmo de Kruskal para encontrar el costo mínimo del árbol de expansión.
    def kruskal(graph):
        edges = [(graph[u][v], u, v) for u in graph for v in graph[u]]
        edges.sort()
        parent = {node: node for node in graph}
        tree = {}
        for cost, u, v in edges:
            if parent[u] != parent[v]:
                tree[u] = tree.get(u, {})
                tree[u][v] = cost
                for node in parent:
                    if parent[node] == parent[v]:
                        parent[node] = parent[u]
        return tree
    
    minimum_spanning_tree = kruskal(graph)
    cost = sum(sum(minimum_spanning_tree[node].values()) for node in minimum_spanning_tree)
    return cost

def a_star_search(graph, start, goal):
    open_list = []  # Cola de prioridad para nodos abiertos.
    closed_set = set()  # Conjunto de nodos cerrados.
    came_from = {}  # Diccionario para rastrear el camino desde el nodo inicial.
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0

    # Inicializar la cola de prioridad con el nodo de inicio y su costo estimado.
    heapq.heappush(open_list, (0 + minimum_spanning_tree_cost(graph), start))

    while open_list:
        _, current = heapq.heappop(open_list)  # Obtiene el nodo con el menor costo estimado desde la cola.

        if current == goal:
            # Reconstruir el camino desde el objetivo hasta el inicio.
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
                f_score = g_score[neighbor] + minimum_spanning_tree_cost(graph)
                if neighbor not in [node[1] for node in open_list]:
                    heapq.heappush(open_list, (f_score, neighbor))

    return None  # No se encontró un camino.

start_node = 'A'  # Nodo de inicio.
goal_node = 'F'   # Nodo objetivo.

result = a_star_search(graph, start_node, goal_node)  # Llamada a la función A* con heurística de costo mínimo del árbol de expansión.

if result:
    print("Camino encontrado:")
    print(result)  # Imprimir la ruta encontrada.
else:
    print("No se encontró un camino.")