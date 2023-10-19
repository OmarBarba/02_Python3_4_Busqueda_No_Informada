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

# Inicialización de la población
def initialize_population(population_size, n):
    population = []
    for _ in range(population_size):
        solution = list(range(n))
        random.shuffle(solution)
        population.append(solution)
    return population

# Selección de padres mediante torneo
def select_parents(population, k):
    parents = random.sample(population, k)
    parents.sort(key=lambda x: objective_function(x, graph))
    return parents[0]

# Cruce de dos soluciones
def crossover(parent1, parent2):
    n = len(parent1)
    start, end = random.sample(range(n), 2)
    if start > end:
        start, end = end, start
    child = [-1] * n
    for i in range(start, end + 1):
        child[i] = parent1[i]
    j = 0
    for i in range(n):
        if child[i] == -1:
            while parent2[j] in child:
                j += 1
            child[i] = parent2[j]
    return child

# Mutación de una solución
def mutate(solution, mutation_rate):
    n = len(solution)
    for i in range(n):
        if random.random() < mutation_rate:
            j = random.randint(0, n - 1)
            solution[i], solution[j] = solution[j], solution[i]

# Algoritmo Genético
def genetic_algorithm(graph, population_size, generations, mutation_rate, tournament_size):
    n = len(graph)
    population = initialize_population(population_size, n)
    
    for _ in range(generations):
        new_population = []

        for _ in range(population_size // 2):
            parent1 = select_parents(population, tournament_size)
            parent2 = select_parents(population, tournament_size)
            child1 = crossover(parent1, parent2)
            child2 = crossover(parent2, parent1)
            mutate(child1, mutation_rate)
            mutate(child2, mutation_rate)
            new_population.extend([child1, child2])

        population = new_population

    best_solution = min(population, key=lambda x: objective_function(x, graph))
    best_cost = objective_function(best_solution, graph)
    return best_solution, best_cost

# Parámetros del Algoritmo Genético
population_size = 100
generations = 100
mutation_rate = 0.01
tournament_size = 5

# Ejecución del Algoritmo Genético
best_solution, best_cost = genetic_algorithm(graph, population_size, generations, mutation_rate, tournament_size)

# Resultados
print("Mejor solución encontrada:", best_solution)
print("Costo de la mejor solución:", best_cost)
