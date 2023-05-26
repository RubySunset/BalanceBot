import math

class MazeGraph:

    def __init__(self):
        self.a_list = {}
        self.internal_path = [] # The path used to navigate the robot to the end once enough of the maze has been discovered.
        self.external_path = [] # The path sent to the web server to display.
        # In the discovery phase, the external path is the shortest path from the start to the robot.
        # In the pathfinding phase, the external path is the shortest path from the start to the end.
        self.path_index = 0 # The next vertex on the path we aim to reach.
    
    # Reset to initial state.
    def reset(self):
        self.a_list = {}
        self.internal_path = []
        self.external_path = []
        self.path_index = 0
    
    # Finds the position of the cell connected to the cell with the given position via the given link.
    def adj_pos(self, pos, link):
        if link == 0:
            return (pos[0], pos[1] - 1)
        elif link == 1:
            return (pos[0] + 1, pos[1])
        elif link == 2:
            return (pos[0], pos[1] + 1)
        elif link == 3:
            return (pos[0] - 1, pos[1])
        else:
            print('adj_pos(): invalid link ' + str(link) + '.')
    
    # Add a vertex to the maze graph.
    def add_vertex(self, pos, links):
        if pos in self.a_list:
            return
        self.a_list[pos] = []
        for i in range(4):
            adj_pos = self.adj_pos(pos, i)
            if adj_pos in self.a_list:
                self.a_list[pos].append(adj_pos)
                self.a_list[adj_pos].append(pos)
    
    # Add a starting node to the graph.
    def add_start(self, pos):
        self.a_list[pos] = []
    
    # Compute the shortest path from given source to destination using Dijkstra's algorithm.
    # TODO replace Dijkstra's algorithm with A* search (?).
    def dijkstra(self, source, dest=None):
        # Construct helper objects.
        distance = {}
        prev = {}
        unvisited = []
        for vertex in self.a_list:
            distance[vertex] = math.inf
            prev[vertex] = None
            unvisited.append(vertex)
        # Construct shortest-distance tree.
        distance[source] = 0
        while len(unvisited) > 0:
            current = unvisited[0]
            if dest != None and current == dest:
                break
            for vertex in unvisited:
                if distance[vertex] < distance[current]:
                    current = vertex
            unvisited.remove(current)
            for neighbour in self.a_list[current]:
                if neighbour not in unvisited:
                    continue
                if current[0] == neighbour[0]:
                    next_hop = abs(current[1] - neighbour[1])
                elif current[1] == neighbour[1]:
                    next_hop = abs(current[0] - neighbour[0])
                else:
                    print('Graph error')
                alt_dist = distance[current] + next_hop
                # alt_dist = distance[current] + math.dist(current, neighbour)
                if alt_dist < distance[neighbour]:
                    distance[neighbour] = alt_dist
                    prev[neighbour] = current
        # Determine shortest path to end node.
        shortest_path = []
        if dest != None:
            node = dest
            while node != None:
                shortest_path.insert(0, node)
                node = prev[node]
        return shortest_path, distance
    
    # Navigate to the next stop on the shortest path.
    def follow_path(self, pos, orientation, end_pos):
        next = self.internal_path[self.path_index]
        if pos == next and next != end_pos:
            self.path_index += 1
            next = self.internal_path[self.path_index]
            if pos[1] > next[1]:
                desired_orientation = 0
            elif pos[0] < next[0]:
                desired_orientation = 1
            elif pos[1] < next[1]:
                desired_orientation = 2
            elif pos[0] > next[0]:
                desired_orientation = 3
            return desired_orientation
        elif pos == next and next == end_pos:
            return 4
        else:
            return orientation
    
    # Find the current distance tree.
    def distance_tree(self, start):
        path, dist = self.dijkstra(start)
        return dist
    
    # Generate the path from the robot's current position to the end, to navigate.
    def generate_remaining_path(self, pos, end):
        path, dist = self.dijkstra(pos, end)
        self.internal_path = path
    
    # Generate the complete shortest path from start to end, to send somewhere.
    def generate_complete_path(self, start, end):
        path, dist = self.dijkstra(start, end)
        self.external_path = path
    
    # Generate the shortest path from start to robot's position, to send somewhere.
    def generate_partial_path(self, start, pos):
        path, dist = self.dijkstra(start, pos)
        self.external_path = path