# Grafo representado como un diccionario de nodos y sus vecinos
graph = {
    'A': ['B', 'C', 'D'],
    'B': ['A', 'C'],
    'C': ['A', 'B', 'D'],
    'D': ['A', 'C']
}

# Inicialización de asignación de colores
coloring = {}

# Función para verificar si una asignación es válida
def is_valid(node, color, coloring):
    for neighbor in graph[node]:
        if neighbor in coloring and coloring[neighbor] == color:
            return False
    return True

# Función de búsqueda hacia adelante
def forward_checking(node, colors, coloring):
    if node not in coloring:
        for color in colors:
            if is_valid(node, color, coloring):
                coloring[node] = color
                if len(coloring) == len(graph):
                    return coloring  # Solución encontrada
                next_node = [n for n in graph if n not in coloring][0]
                result = forward_checking(next_node, colors, coloring)
                if result is not None:
                    return result
                del coloring[node]  # Retroceder
    return None

# Colores disponibles
available_colors = ['Red', 'Green', 'Blue']

# Iniciar la búsqueda desde el primer nodo
initial_node = list(graph.keys())[0]
solution = forward_checking(initial_node, available_colors, coloring)

if solution is not None:
    print("Asignación de colores válida:")
    for node, color in solution.items():
        print(f"Nodo {node}: {color}")
else:
    print("No se encontró una asignación de colores válida.")
