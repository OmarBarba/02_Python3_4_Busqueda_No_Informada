import random

# Definición de la función objetivo (puede ser cualquier función)
def objective_function(x):
    return -x**2 + 10

# Función de Búsqueda de Ascensión de Colinas
def hill_climbing(iterations, step_size, initial_state):
    current_state = initial_state
    for _ in range(iterations):
        next_state = current_state + random.uniform(-step_size, step_size)
        if objective_function(next_state) > objective_function(current_state):
            current_state = next_state
    return current_state, objective_function(current_state)

# Parámetros de la búsqueda
iterations = 1000
step_size = 0.1
initial_state = 5.0  # Estado inicial aleatorio

# Ejecución de la búsqueda de ascensión de colinas
final_state, maximum = hill_climbing(iterations, step_size, initial_state)

# Resultados
print("Estado final encontrado:", final_state)
print("Valor máximo encontrado:", maximum)


# Ejemplo de un grafo representado como un diccionario de adyacencia
graph = {
    'A': ['B', 'C'],
    'B': ['A', 'D', 'E'],
    'C': ['A', 'F'],
    'D': ['B'],
    'E': ['B', 'F'],
    'F': ['C', 'E']
}

# Función objetivo: Longitud del camino
def objective_function(path):
    return len(path)

# Búsqueda de Ascensión de Colinas en el grafo
def hill_climbing(graph, start, goal):
    current_node = start
    current_path = [current_node]

    while current_node != goal:
        neighbors = graph[current_node]
        candidate_neighbors = [n for n in neighbors if len(current_path) < len(graph[n])]
        
        if not candidate_neighbors:
            break

        next_node = random.choice(candidate_neighbors)
        current_node = next_node
        current_path.append(current_node)

    return current_path

# Parámetros de la búsqueda
start_node = 'A'
goal_node = 'F'

# Ejecución de la búsqueda de ascensión de colinas
path = hill_climbing(graph, start_node, goal_node)

# Resultados
if path:
    print("Camino encontrado:", path)
    print("Longitud del camino:", objective_function(path))
else:
    print("No se encontró un camino.")
