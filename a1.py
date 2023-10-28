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


# Example usage:
rows = 40
cols = 40
obstacle_density = 0.4
start = (2, 3)
destination = (38, 39)
grid = create_grid(rows, cols,start,destination,obstacle_density)    # Define the grid with obstacles, start, and destination
for row in grid:
    print(row)

path, opened_nodes, blocked_path, cost = astar_search(grid, start, destination)
print("cost",cost)

if path:
    print("Optimal Path:", path)
    visualize_path_with_animation(grid, path, opened_nodes, blocked_path)
else:
    print("No path found.")

