import networkx as nx
import matplotlib.pyplot as plt
import math
from queue import PriorityQueue

# --- Heuristic function (straight-line distance) ---


def heuristic(a, b, pos):
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

# --- A* Algorithm Implementation ---


def astar(graph, start, goal, pos):
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    g_score = {node: float('inf') for node in graph.nodes}
    g_score[start] = 0

    while not open_set.empty():
        _, current = open_set.get()

        if current == goal:
            # reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # reverse

        for neighbor in graph.neighbors(current):
            tentative_g = g_score[current] + graph[current][neighbor]['weight']
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal, pos)
                open_set.put((f_score, neighbor))
    return None


# --- Build Larger Graph ---
G = nx.Graph()

# Expanded positions (x,y)
pos = {
    'Rathmalana': (0, 0),
    'Panadura': (2, 1),
    'Kalutara': (4, 0),
    'Homagama': (1, 3),
    'Horana': (3, 3),
    'Colombo': (-2, 1),
    'Kottawa': (0, 2),
    'Moratuwa': (1, -1),
    'Matugama': (6, 1),
    'Bandaragama': (2, 2),
    'Piliyandala': (1, 1),
    'Bulathsinhala': (5, 3)
}

# Expanded edges with weights (distances)
edges = [
    ('Colombo', 'Rathmalana', 12), ('Colombo', 'Kottawa', 15),
    ('Rathmalana', 'Moratuwa', 5), ('Rathmalana', 'Kottawa', 10),
    ('Moratuwa', 'Panadura', 10),
    ('Panadura', 'Kalutara', 15),  ('Panadura', 'Bandaragama', 12),
    ('Kalutara', 'Matugama', 18), ('Kalutara',
                                   'Horana', 28), ('Kalutara', 'Bulathsinhala', 22),
    ('Homagama', 'Kottawa', 8), ('Homagama',
                                 'Horana', 20), ('Homagama', 'Piliyandala', 14),
    ('Horana', 'Bandaragama', 10), ('Horana', 'Bulathsinhala', 12),
    ('Matugama', 'Bulathsinhala', 15), ('Piliyandala', 'Bandaragama', 7),
    ('Kottawa', 'Piliyandala', 9)
]

G.add_weighted_edges_from(edges)

# --- Run A* ---
start, goal = 'Rathmalana', 'Horana'
path = astar(G, start, goal, pos)
print("Shortest path:", path)

# --- Draw Graph ---
plt.figure(figsize=(10, 7))
nx.draw(G, pos, with_labels=True, node_size=2500,
        node_color='lightblue', font_size=10, font_weight='bold')
nx.draw_networkx_edge_labels(G, pos, edge_labels={(
    u, v): d['weight'] for u, v, d in G.edges(data=True)}, font_size=8)
nx.draw_networkx_edges(G, pos, edgelist=list(
    zip(path, path[1:])), width=3, edge_color='r')
plt.show()
