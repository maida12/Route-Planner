import random
import numpy as np
import heapq
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class Node:
    def __init__(self, x, y, cost=0, heuristic=0):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.parent = None  # Initialize the parent attribute

    def __lt__(self, other): 
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)

def get_neighbors(grid, node):
    neighbors = []
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)]
    for dx, dy in directions:
        new_x, new_y = node.x + dx, node.y + dy

        if 0 <= new_x < len(grid) and 0 <= new_y < len(grid[0]) and grid[new_x][new_y] != 1:
            neighbors.append(Node(new_x, new_y))

    return neighbors

def heuristic(node, destination):
    return abs(node.x - destination[0]) + abs(node.y - destination[1])

def astar_search(grid, start, destination):
    start_node = Node(start[0], start[1])
    goal_node = Node(destination[0], destination[1])

    open_set = [start_node]
    closed_set = set()
    opened_nodes = []  # Track opened nodes
    blocked_path = []  # Track blocked path
    tentative_cost = 0
    while open_set:
        current_node = heapq.heappop(open_set)

        if (current_node.x, current_node.y) == (goal_node.x, goal_node.y):
            # Goal reached, reconstruct path
            path = []
            while current_node:
                path.insert(0, (current_node.x, current_node.y))
                current_node = current_node.parent
            print ("A* Path:", path)
            return path, opened_nodes, blocked_path, tentative_cost  # Include total cost

        closed_set.add((current_node.x, current_node.y))

        for neighbor in get_neighbors(grid, current_node):
            if (neighbor.x, neighbor.y) in closed_set:
                continue

            tentative_cost = current_node.cost + 1
            if neighbor not in open_set or tentative_cost < neighbor.cost:
                neighbor.parent = current_node
                neighbor.cost = tentative_cost
                neighbor.heuristic = heuristic(neighbor, destination)
                heapq.heappush(open_set, neighbor)
                opened_nodes.append((neighbor.x, neighbor.y))

        # If the neighbor is not in open_set, it's part of the blocked path
        if (neighbor.x, neighbor.y) not in open_set:
            blocked_path.append((neighbor.x, neighbor.y))

    return None, opened_nodes, blocked_path, 0  # No path found, total cost is 0

def create_grid(rows, cols,start, destination,obstacle_density=0.5):
  
    grid = [[0 for i in range(cols)] for j in range(rows)]

    # Place obstacles randomly based on obstacle density
    for row in range(rows):
        for col in range(cols):
            if random.random() < obstacle_density:
                grid[row][col] = 1  # Mark as obstacle
    grid[start[0]][start[1]]='x'
    grid[destination[0]][destination[1]]='x'
    return grid

def update_animation(frame, ax, path, opened_nodes, blocked_path):
    ax.clear()

    # Plot obstacles in the grid
    for i in range(rows):
        for j in range(cols):
            if grid[i][j] == 1:
                ax.add_patch(plt.Rectangle((j, rows - 1 - i), 1, 1, color='gray'))

    # Plot the blocked path
    for node in blocked_path:
        ax.add_patch(plt.Rectangle((node[1], rows - 1 - node[0]), 1, 1, color='red'))

    # Plot the opened nodes
    for node in opened_nodes:
        ax.add_patch(plt.Rectangle((node[1], rows - 1 - node[0]), 1, 1, color='orange'))

    # Plot the path
    for node in path[:frame]:
        ax.add_patch(plt.Rectangle((node[1], rows - 1 - node[0]), 1, 1, color='green'))

    # Plot the ambulance at its current position
    current_position = path[frame]
    ax.add_patch(plt.Rectangle((current_position[1], rows - 1 - current_position[0]), 1, 1, color='blue'))

    # Set axis limits and labels
    ax.set_xlim(0, cols)
    ax.set_ylim(0, rows)
    ax.set_xticks(range(cols))
    ax.set_yticks(range(rows))
    ax.set_xticklabels([])
    ax.set_yticklabels([])

def visualize_path_with_animation(grid, path, opened_nodes, blocked_path):
    rows = len(grid)
    cols = len(grid[0])

    # Create a figure and axis
    fig, ax = plt.subplots()

    # Create the animation
    animation = FuncAnimation(
        fig, update_animation, fargs=(ax, path, opened_nodes, blocked_path),
        frames=len(path), repeat=False
    )

    # Show the plot
    plt.show()


class Individual:
    def __init__(self, genes):
        self.genes = genes
        self.fitness = 0
    def __str__(self):
        return f"Genes: {self.genes}, Fitness: {self.fitness}"

def create_individual(grid, start, destination, path_length=5):
    rows, cols = len(grid), len(grid[0])
    path = [start]

    for _ in range(path_length - 1):
        current_node = path[-1]
        neighbors = [(path[-1][0] + dx, path[-1][1] + dy) for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]]
        valid_neighbors = [(x, y) for x, y in neighbors if 0 <= x < rows and 0 <= y < cols and grid[x][y] != 1]
        
        if valid_neighbors:
            next_step = random.choice(valid_neighbors)
            path.append(next_step)
            path=random_path(current_node, path[-1], grid,path)
        else:
            # If no valid neighbors, stop generating the path
            break
    path.append(destination)  # Ensure the path reaches the destination
    path=random_path(path[-2],destination, grid,path)
    
    unique_list = []
    for item in path:
        if item not in unique_list:
            unique_list.append(item)

    

    # print("path",unique_list)
    return Individual(unique_list)

def random_path(start, destination, grid,path):
    
    path1, _, _, _   = astar_search(grid, start, destination)
    value_to_insert_before = destination

    # Find the index of the value
    index_to_insert = path.index(value_to_insert_before)
    result_array = path[:-1] + path1 + path[-1:]
    return result_array



def crossover(parent1, parent2):
    # Perform crossover to create a new individual
    crossover_point = random.randint(1, min(len(parent1.genes), len(parent2.genes)) - 1)
    genes = parent1.genes[:crossover_point] + parent2.genes[crossover_point:]
    return Individual(genes)

def mutate(individual, mutation_rate):
    # Perform mutation on an individual
    for i in range(1, len(individual.genes)):
        if random.random() < mutation_rate:
            value=random.choice(get_neighbors(grid, Node(*individual.genes[i])))
            individual.genes[i] = value.x, value.y
    return individual

def fitness_function(individual, destination):
    # Fitness function - the lower the total cost, the better
    total_cost = 0
    for i in range(1, len(individual.genes)):
        total_cost += heuristic(Node(individual.genes[i-1][0],individual.genes[i-1][1]), destination)
    
    individual.fitness = total_cost
    return total_cost

def genetic_algorithm(population_size, generations, mutation_rate, start, destination, grid):
    population = [create_individual(grid,start, destination,path_length=5) for _ in range(population_size)]
   
    for generation in range(generations):
        print("====generation====",generation)
         # Evaluate fitness for each individual in the population
        for individual in population:
            individual.fitness = fitness_function(individual, destination)

        # Select parents based on fitness
        parents = sorted(population, key=lambda x: x.fitness)[:2]
        for individual in parents:
            print("pp",individual)

        # Create offspring through crossover and mutation
        offspring = [crossover(parents[0], parents[1]) for _ in range(population_size - 2)]
        for individual in offspring:
            print("ppyp",individual)
        offspring = [mutate(ind, mutation_rate) for ind in offspring]
        for individual in offspring:
            print("ppjyp",individual)

        # Replace the old population with the new generation
        population = parents + offspring
        for individual in population:
            print("ppp",individual)

    # Select the best route from the final population
    best_route = min(population, key=lambda x: x.fitness)
    print("best_routedd",best_route)
    return best_route



# # Example usage:
# rows = 10
# cols = 10
# obstacle_density = 0.4
# start = (2, 3)
# destination = (8, 9)
# grid = create_grid(rows, cols,start,destination,obstacle_density)    # Define the grid with obstacles, start, and destination
# for row in grid:
#     print(row)


# # Run A* to get a reference optimal path
# astar_path = astar_search(grid, start, destination)

# # Run Genetic Algorithm
# genetic_path = genetic_algorithm(population_size=10, generations=100, mutation_rate=0.2,
#                                   start=start, destination=destination, grid=grid)

# print("A* Path:", astar_path)
# print("Genetic Algorithm Path:", genetic_path)


# import random
# import numpy as np

# class Chromosome:
#     def __init__(self, path):
#         self.path = path
#         self.fitness = None

#     def evaluate_fitness(self, grid, start, destination):
#         # Calculate the length of the path
#         path_length = 0
#         for i in range(len(self.path) - 1):
#             path_length += grid[self.path[i][0]][self.path[i][1]]

#         # Calculate the number of obstacles avoided
#         num_obstacles_avoided = 0
#         for node in self.path:
#             if grid[node[0]][node[1]] == 1:
#                 num_obstacles_avoided += 1

#         # Set the fitness of the chromosome
#         self.fitness = path_length - num_obstacles_avoided

#     def crossover(self, other_chromosome):
#         # Choose a random crossover point
#         crossover_point = random.randint(1, len(self.path) - 1)

#         # Create two child chromosomes
#         child1 = Chromosome(self.path[:crossover_point] + other_chromosome.path[crossover_point:])
#         child2 = Chromosome(other_chromosome.path[:crossover_point] + self.path[crossover_point:])

#         return child1, child2

#     def mutate(self, mutation_rate):
#         for i in range(len(self.path)):
#             if random.random() < mutation_rate:
#                 # Swap the node at position i with a random node
#                 random_node = random.randint(0, len(self.path) - 1)
#                 self.path[i], self.path[random_node] = self.path[random_node], self.path[i]

# def genetic_algorithm(grid, start, destination, population_size, num_generations, mutation_rate):
#     # Initialize the population
#     population = []
#     for i in range(population_size):
#         startnode=Node(start[0],start[1])
#         goalnode=Node(destination[0],destination[1])
#         path = [startnode]
#         while ((path[-1].x,path[-1].y) != (goalnode.x, goalnode.y)):
#             # Choose a random neighbor of the current node
#             neighbor = random.choice(get_neighbors(grid, path[-1]))
#             path.append(neighbor)

#         chromosome = Chromosome(path)
#         population.append(chromosome)
#         print("chromosome",chromosome)
#         print("  ")
#         print("population",population)

#     # Evaluate the fitness of each chromosome
#     for chromosome in population:
#         chromosome.evaluate_fitness(grid, start, destination)

#     # Repeat until a termination condition is met
#     for generation in range(num_generations):
#         # Select parents
#         parents = []
#         for i in range(2):
#             parents.append(random.choices(population, weights=[chromosome.fitness for chromosome in population])[0])

#         # Crossover the parents to create new child chromosomes
#         child1, child2 = parents[0].crossover(parents[1])

#         # Mutate the child chromosomes
#         child1.mutate(mutation_rate)
#         child2.mutate(mutation_rate)
#         # Replace the worst chromosomes in the population with the new child chromosomes
#         population.sort(key=lambda chromosome: chromosome.fitness, reverse=True)
#         population.pop()
#         population.pop()
#         population.append(child1)
#         population.append(child2)

#     # Return the best chromosome
#     return max(population, key=lambda chromosome: chromosome.fitness)


rows = 10
cols = 10
obstacle_density = 0.4
start = (2, 3)
destination = (8, 9)
population_size=5
num_generations=4
mutation_rate=0.2
grid = create_grid(rows, cols,start,destination,obstacle_density)    # Define the grid with obstacles, start, and destination
for row in grid:
    print(row)

path, opened_nodes, blocked_path, cost = astar_search(grid, start, destination)
print("cost",cost)

genetic_path = genetic_algorithm(population_size=5, generations=10, mutation_rate=0.2,
                                  start=start, destination=destination, grid=grid)

print("Genetic Algorithm Path:", genetic_path)
