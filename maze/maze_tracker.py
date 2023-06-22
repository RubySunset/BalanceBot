import math
import time
import random

# Main class. Instantiate and use in other modules.
class MazeTracker:

    def __init__(self, MIN_DIST):
        self.MIN_DIST = MIN_DIST # The minimum distance between two vertices in metres.
        self.a_list = {} # Adjacency list to store the maze graph. Vertices are stored by their positions.
        # self.marks = {} # A mapping from vertex positions to lists of link marks.
        self.num_links = {} # The total number of links for each vertex.
        self.explored_links = {} # The number of explored links for each vertex.
        self.prev_vertex = None # The position of the last vertex reached.
        self.start = None # Start position.
        self.end = None # End position.
        self.external_path = [] # The path sent to the web server to display.
        # When the robot is still traversing the maze, this is the shortest path from the start to the robot.
        # When the robot has reached the end and has surveyed the entire maze, this is the shortest path from start to end.
        self.prev_entry_link = None
    
    # Reset to initial state.
    def reset(self):
        self.a_list = {}
        # self.marks = {}
        self.num_links = {}
        self.explored_links = {}
        self.prev_vertex = None
        self.start = None
        self.end = None
        self.external_path = []
        self.prev_entry_link = None
    
    # Finds the difference between two numbers in modular arithmetic.
    def mod_diff(self, a, b, mod):
        diff = b - a
        if diff <= -mod/2:
            return diff + mod
        elif diff > mod/2:
            return diff - mod
        else:
            return diff
    
    # Finds the average of two numbers in modular arithmetic.
    def mod_avg(self, a, b, mod):
        # if a > 180:
        #     x = a - 360
        # else:
        #     x = a
        # if b > 180:
        #     y = b - 360
        # else:
        #     y = b
        # return (x + y) / 2
        diff = self.mod_diff(a, b, mod)
        if diff > 0:
            return (a + diff/2) % 360
        else:
            return (a - diff/2) % 360
    
    # Set a starting point.
    def set_start(self, pos):
        self.start = pos
    
    # Set an end point.
    def set_end(self, pos):
        self.end = pos
    
    # Find the distance tree from the given source vertex (and optinally an end vertex) using Dijkstra's algorithm.
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
            for vertex in unvisited:
                if distance[vertex] < distance[current]:
                    current = vertex
            if dest != None and current == dest:
                break
            unvisited.remove(current)
            for neighbour in self.a_list[current]:
                if neighbour not in unvisited:
                    continue
                alt_dist = distance[current] + math.dist(current, neighbour)
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
        return distance, shortest_path
    
    # Try to find a vertex near enough to the given position.
    def find_vertex(self, pos):
        closest_dist = math.inf
        closest_v = None
        for v in self.a_list:
            dist = math.dist(v, pos)
            if dist <= self.MIN_DIST and dist < closest_dist:
                closest_dist = dist
                closest_v = v
        return closest_v
    
    # Replace all instances of one vertex with another.
    def replace_vertex(self, old_v, new_v):
        self.a_list[new_v] = self.a_list.pop(old_v)
        self.num_links[new_v] = self.num_links.pop(old_v)
        self.explored_links[new_v] = self.explored_links.pop(old_v)
        for v in self.a_list:
            if old_v in self.a_list[v]:
                self.a_list[v].remove(old_v)
                self.a_list[v].add(new_v)

    # Visit a vertex, updating the relevant graph structures.
    # Assume all angles are taken clockwise from north in the range [0, 360].
    def visit_vertex(self, pos, link_angles):
        if self.prev_vertex == None: # If we are at the first iteration.
            self.a_list[self.start] = set()
            self.num_links[self.start] = len(link_angles)
            self.explored_links[self.start] = 0
            return self.start
        last_vertex = self.prev_vertex
        # b = math.dist(pos, self.prev_vertex)
        # dist = b
        # for v in self.a_list:
        #     if v in (self.prev_vertex, pos):
        #         continue
        #     c = math.dist(pos, v)
        #     a = math.dist(self.prev_vertex, v)
        #     alpha = math.degrees(math.acos((b**2 + c**2 - a**2) / (2*b*c)))
        #     d = math.dist(pos, v)
        #     if d < dist and alpha <= 10:
        #         last_vertex = v
        #         dist = d
        if math.dist(pos, self.start) <= self.MIN_DIST:
            if last_vertex != self.start:
                if last_vertex not in self.a_list[self.start]:
                    self.explored_links[self.start] += 1
                    self.explored_links[last_vertex] += 1
                self.a_list[last_vertex].add(self.start)
                self.a_list[self.start].add(last_vertex)
            return self.start
        elif math.dist(pos, self.end) <= self.MIN_DIST:
            if last_vertex != self.end:
                if self.end in self.a_list:
                    if last_vertex not in self.a_list[self.end]:
                        self.explored_links[self.end] += 1
                        self.explored_links[last_vertex] += 1
                    self.a_list[last_vertex].add(self.end)
                    self.a_list[self.end].add(last_vertex)
                else:
                    self.a_list[last_vertex].add(self.end)
                    self.a_list[self.end] = {last_vertex}
                    self.num_links[self.end] = len(link_angles)
                    self.explored_links[self.end] = 1
                    self.explored_links[last_vertex] += 1
            return self.end
        elif math.dist(pos, last_vertex) <= self.MIN_DIST: # If we are at the same vertex.
            avg_pos = (round((pos[0] + last_vertex[0]) / 2, 3), round((pos[1] + last_vertex[1]) / 2, 3))
            self.replace_vertex(last_vertex, avg_pos)
            return avg_pos
        else:
            near_v = self.find_vertex(pos)
            if near_v == None:
                self.a_list[last_vertex].add(pos)
                self.a_list[pos] = {last_vertex}
                self.num_links[pos] = len(link_angles)
                self.explored_links[pos] = 1
                self.explored_links[last_vertex] += 1
                return pos
            else:
                avg_pos = (round((near_v[0] + pos[0]) / 2, 3), round((near_v[1] + pos[1]) / 2, 3))
                self.replace_vertex(near_v, avg_pos)
                if last_vertex not in self.a_list[avg_pos]:
                    self.explored_links[avg_pos] += 1
                    self.explored_links[last_vertex] += 1
                self.a_list[last_vertex].add(avg_pos)
                self.a_list[avg_pos].add(last_vertex)
                return avg_pos
    
    # Navigate during the discovery phase. Also apply an exit mark.
    def discovery_navigate(self, pos, link_angles):
        if self.num_links[pos] != len(link_angles):
            print('!!! Warning: current number of links (' + str(len(link_angles)) + ') does not match previously found number of links (' + str(self.num_links[pos]) + ').')
            # Assume that the greater number of links is correct, for safety.
            if len(link_angles) > self.num_links[pos]:
                self.num_links[pos] = len(link_angles)
        assert self.explored_links[pos] == len(self.a_list[pos])
        if self.explored_links[pos] > self.num_links[pos]:
            print('!!! Warning: number of explored links exceeded total number of links.')
        if self.explored_links[pos] >= self.num_links[pos]:
            tree, path = self.dijkstra(pos)
            min_dist = math.inf
            min_pos = None
            for v in self.a_list:
                if self.explored_links[v] < self.num_links[v] and tree[v] < min_dist:
                    min_dist = tree[v]
                    min_pos = v
            if min_pos == None:
                return random.random() * 360
            else:
                tree, path = self.dijkstra(pos, min_pos)
                target_pos = path[1]
                diff = (target_pos[0] - pos[0], pos[1] - target_pos[1])
                target_angle = (90 - math.degrees(math.atan2(diff[1], diff[0]))) % 360
                if len(link_angles) == self.num_links[pos]:
                    target_link = 0
                    for i in range(len(link_angles)):
                        if abs(self.mod_diff(link_angles[i], target_angle, 360)) < abs(self.mod_diff(link_angles[target_link], target_angle, 360)):
                            target_link = i
                    # avg_angle = (target_angle + link_angles[target_link]) / 2
                    return self.mod_avg(target_angle, link_angles[target_link], 360)
                else:
                    return target_angle
        elif len(link_angles) < self.num_links[pos]:
            return random.random() * 360
        else:
            unexplored_angles = []
            for angle in link_angles:
                unexplored_angles.append(angle)
            for v in self.a_list[pos]:
                diff = (v[0] - pos[0], pos[1] - v[1])
                true_angle = (90 - math.degrees(math.atan2(diff[1], diff[0]))) % 360
                closest_link = 0
                for i in range(len(link_angles)):
                    if abs(self.mod_diff(link_angles[i], true_angle, 360)) < abs(self.mod_diff(link_angles[closest_link], true_angle, 360)):
                        closest_link = i
                unexplored_angles.remove(link_angles[closest_link])
            return unexplored_angles[0]
    
    # Navigate after the discovery phase using Dijkstra.
    def dijkstra_navigate(self, pos):
        tree, path = self.dijkstra(pos, self.end)
        target_pos = path[1]
        # diff = [target_pos[i] - pos[i] for i in range(2)]
        diff = (target_pos[0] - pos[0], pos[1] - target_pos[1])
        return (90 - math.degrees(math.atan2(diff[1], diff[0]))) % 360
    
    # Generate the complete shortest path from start to end, for the front-end to display
    # once the robot has finished the discovery phase.
    def generate_complete_path(self):
        tree, self.external_path = self.dijkstra(self.start, self.end)
    
    # Generate the shortest path from start to robot's position, for the front-end to display
    # while the robot is still in the discovery phase.
    def generate_partial_path(self, disc_pos, cont_pos):
        tree, self.external_path = self.dijkstra(self.start, disc_pos)
        self.external_path.append(cont_pos)
    
    # Update the external path with the robot's current position.
    def update_partial_path(self, pos):
        self.external_path.pop()
        self.external_path.append(pos)

    # Test to see if we have discovered enough of the maze to determine the shortest path.
    def enough_discovered(self):
        dist_tree, path = self.dijkstra(self.start, self.end)
        for v in self.a_list:
            if self.explored_links[v] < self.num_links[v] and dist_tree[v] + math.dist(v, self.end) < dist_tree[self.end]:
                # If there could exist a shorter path to the end via this vertex.
                return False
        print('Sufficient portion of maze discovered!')
        return True