from constraint import Problem

# Crear una instancia del problema
problem = Problem()

# Nodos del grafo
nodes = ['A', 'B', 'C', 'D']

# Colores disponibles
colors = ['Rojo', 'Verde', 'Azul']

# Agregar variables al problema (una para cada nodo)
for node in nodes:
    problem.addVariable(node, colors)

# Agregar restricciones (no se deben usar el mismo color en nodos adyacentes)
for node in nodes:
    for neighbor in nodes:
        if neighbor in graph[node]:
            problem.addConstraint(lambda color1, color2: color1 != color2, (node, neighbor))

# Encontrar una solución
solutions = problem.getSolutions()

if solutions:
    for solution in solutions:
        print("Solución encontrada:")
        for node, color in solution.items():
            print(f"Nodo {node}: {color}")
else:
    print("No se encontró una asignación de colores válida.")
