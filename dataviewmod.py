from collections import deque
import turtle
import heapq

# Global variables
nodes = {
    "3": (300, 200),
    "1": (100, 200),
    "2": (100, 100),
    "4": (300, 100),
    "5": (100, 0),
    "6": (300, 0),
    "7": (100, -100),
    "8": (300, -100),
    "9": (-100, -100),
    "10": (-100, 0),
    "11": (-100, 100),
    "12": (-400, 0),
    "13": (-400, -100),
    "14": (-300, -100),
    "15": (-400, 100),
    "16": (-300, -50)
}

edges = {
    "1": ["2", "3"],
    "2": ["1", "4", "5", "11"],
    "3": ["1", "4"],
    "4": ["3", "2", "6"],
    "5": ["2", "6", "7", "10"],
    "6": ["4", "5", "8"],
    "7": ["5", "8", "9"],
    "8": ["6", "7"],
    "9": ["7", "10", "14"],
    "10": ["9", "12", "11", "5"],
    "11": ["10", "2"],
    "14": ["9", "13", "16"],
    "13": ["14", "12"],
    "12": ["13", "15", "10"],
    "16": ["14"],
    "15": ["12"]
}

obstacles = set()
path_index = 0
moving = True
path = []
start_node = ""
goal_node = ""
step_size = 5  # Distance to move in each step


# Function to draw the map
def draw_map():
    drawer = turtle.Turtle()
    drawer.speed("fastest")
    drawer.penup()
    for node, position in nodes.items():
        drawer.goto(position)
        drawer.dot(20, "blue")
        drawer.write(node, align="center", font=("Arial", 12, "normal"))

    drawer.pencolor("black")
    drawer.pensize(2)
    for node, neighbors in edges.items():
        start_pos = nodes[node]
        for neighbor in neighbors:
            end_pos = nodes[neighbor]
            drawer.goto(start_pos)
            drawer.pendown()
            drawer.goto(end_pos)
            drawer.penup()

    drawer.hideturtle()


# Heuristic function for A* (Manhattan distance)
def heuristic(node1, node2):
    x1, y1 = nodes[node1]
    x2, y2 = nodes[node2]
    return abs(x1 - x2) + abs(y1 - y2)


# A* pathfinding algorithm
def a_star(start, goal, avoid_edge=None):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {node: float("inf") for node in nodes}
    g_score[start] = 0
    f_score = {node: float("inf") for node in nodes}
    f_score[start] = heuristic(start, goal)

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for neighbor in edges[current]:
            if (current, neighbor) == avoid_edge or (neighbor, current) == avoid_edge:
                continue
            if neighbor in obstacles:
                continue
            tentative_g_score = g_score[current] + 1

            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None


# Function to move incrementally towards a target position
def move_towards_target():
    global path_index, moving, path

    if path_index < len(path) and moving:
        current_node = path[path_index]
        target_position = nodes[current_node]
        current_position = agv.position()

        dx = target_position[0] - current_position[0]
        dy = target_position[1] - current_position[1]
        distance = (dx ** 2 + dy ** 2) ** 0.5

        if distance <= step_size:
            agv.goto(target_position)
            path_index += 1
            if current_node == goal_node:
                agv.dot(10, "green")
                # Close the turtle window after completion
                turtle.bye()
                return
        else:
            angle = agv.towards(target_position)
            agv.setheading(angle)
            agv.forward(step_size)

        screen.ontimer(move_towards_target, 10)


# Function to move back to the previous node and stop
def backtrack():
    global path_index, moving, path

    if path_index > 0:
        previous_node = path[path_index - 1]
        target_position = nodes[previous_node]
        current_position = agv.position()

        dx = target_position[0] - current_position[0]
        dy = target_position[1] - current_position[1]
        distance = (dx ** 2 + dy ** 2) ** 0.5

        if distance <= step_size:
            agv.goto(target_position)
            path_index -= 1
            # Recalculate path from the new position, avoiding the specific edge
            new_start = path[path_index]
            avoid_edge = (path[path_index], path[path_index + 1]) if path_index + 1 < len(path) else None
            new_path = a_star(new_start, goal_node, avoid_edge=avoid_edge)
            if new_path and new_path != path:
                path = new_path
                path_index = 0
                start_movement()
            else:
                print("No alternative path found")
                moving = False
                return
        else:
            angle = agv.towards(target_position)
            agv.setheading(angle)
            agv.forward(step_size)
            screen.ontimer(backtrack, 10)
    else:
        moving = False


def stop_movement():
    global moving
    moving = False
    backtrack()


def start_movement():
    global moving
    moving = True
    move_towards_target()


# Main function to setup and run the AGV Path Tracker
def main():
    global screen, agv, path, start_node, goal_node

    screen = turtle.Screen()
    screen.title("AGV Path Tracker")
    screen.setup(width=800, height=600)

    # Get user input for start and goal nodes using command prompt
    global start_node, goal_node
    start_node = input("Enter the start node: ")
    goal_node = input("Enter the goal node: ")

    if start_node not in nodes or goal_node not in nodes:
        print("Invalid start or goal node")
        turtle.bye()  # Close the turtle window
        return

    agv = turtle.Turtle()
    agv.shape("turtle")
    agv.speed("slowest")
    agv.penup()
    draw_map()
    path = a_star(start_node, goal_node)
    if path:
        move_towards_target()
    else:
        print("No path found")
        turtle.bye()  # Close the turtle window
        return

    screen.listen()
    screen.onkey(stop_movement, "q")
    screen.onkey(start_movement, "w")
    screen.mainloop()


class Graph:
    def __init__(self, V):
        self.V = V
        self.adj = [[] for _ in range(2 * self.V)]
        self.level = 0
        self.distances = {}  # Dictionary to store distances between nodes

    def add_edge(self, v, w, weight):
        if weight == 2:
            self.adj[v].append(v + self.V)
            self.adj[v + self.V].append(w)
        else:  # Weight is 1
            self.adj[v].append(w)
        # Store distances between nodes
        self.distances[(v, w)] = weight

    def print_shortest_path(self, parent, s, d):
        self.level = 0
        if parent[s] == -1:
            return [s]

        path = self.print_shortest_path(parent, parent[s], d)
        path.append(s)
        return path

    def find_shortest_path(self, src, dest):
        visited = [False] * (2 * self.V)
        parent = [-1] * (2 * self.V)
        queue = deque()
        visited[src] = True
        queue.append(src)

        while queue:
            s = queue.popleft()
            if s == dest:
                return self.print_shortest_path(parent, s, dest)

            for i in self.adj[s]:
                if not visited[i]:
                    visited[i] = True
                    queue.append(i)
                    parent[i] = s
        return []

    def get_distance(self, node1, node2):
        # Get distance between two nodes from the distances dictionary
        return self.distances.get((node1, node2), None)


V = 22
g = Graph(V)
g.add_edge(12, 11, 40)
g.add_edge(11, 12, 40)
g.add_edge(11, 10, 36)
g.add_edge(10, 11, 36)
g.add_edge(10, 9, 27)
g.add_edge(9, 10, 27)
g.add_edge(9, 8, 47)
g.add_edge(8, 9, 47)
g.add_edge(8, 7, 33)
g.add_edge(7, 8, 33)
g.add_edge(7, 6, 22)
g.add_edge(6, 7, 22)
g.add_edge(6, 5, 14)
g.add_edge(5, 6, 14)
g.add_edge(5, 4, 37)
g.add_edge(4, 5, 37)
g.add_edge(4, 3, 12)
g.add_edge(3, 4, 12)
g.add_edge(3, 2, 45)
g.add_edge(2, 3, 45)
g.add_edge(2, 1, 78)
g.add_edge(1, 2, 78)

if __name__ == "__main__":
    main()