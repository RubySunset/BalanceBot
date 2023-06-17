import math
import time

# Main class. Instantiate and use in other modules.
class MazeTracker:

    MIN_DIST = 0.3 # The minimum distance between two vertices in metres.

    def __init__(self):
        self.a_list = {} # Adjacency list to store the maze graph. Vertices are stored by their positions.
        self.marks = {} # A mapping from vertex positions to lists of link marks.
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
        self.marks = {}
        self.prev_vertex = None
        self.start = None
        self.end = None
        self.external_path = []
        self.prev_entry_link = None
    
    # Finds the difference between two numbers in modular arithmetic.
    def mod_diff(self, a, b, mod):
        diff = abs(a - b)
        if diff < mod/2:
            return diff
        else:
            return mod - diff
    
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
        self.marks[new_v] = self.marks.pop(old_v)
        for v in self.a_list:
            if old_v in self.a_list[v]:
                self.a_list[v].remove(old_v)
                self.a_list[v].add(new_v)

    # Visit a vertex, updating the relevant graph structures.
    # Assume all angles are taken clockwise from north in the range [0, 360].
    def visit_vertex(self, pos, link_angles):
        
        if self.prev_vertex == None: # If we are at the first iteration.
            self.a_list[self.start] = set()
            self.marks[self.start] = []
            for i in range(len(link_angles)):
                self.marks[self.start].append(0)
            return self.start
        elif math.dist(pos, self.start) <= self.MIN_DIST:
            if self.prev_vertex != self.start:
                self.a_list[self.prev_vertex].add(self.start)
                self.a_list[self.start].add(self.prev_vertex)
            return self.start
        elif math.dist(pos, self.end) <= self.MIN_DIST:
            if self.prev_vertex != self.end:
                if self.end in self.a_list:
                    self.a_list[self.prev_vertex].add(self.end)
                    self.a_list[self.end].add(self.prev_vertex)
                else:
                    self.a_list[self.prev_vertex].add(self.end)
                    self.a_list[self.end] = {self.prev_vertex}
                    self.marks[self.end] = []
                    for i in range(len(link_angles)):
                        self.marks[self.end].append(0)
            return self.end
        elif math.dist(pos, self.prev_vertex) <= self.MIN_DIST: # If we are at the same vertex.
            avg_pos = (round((pos[0] + self.prev_vertex[0]) / 2, 3), round((pos[1] + self.prev_vertex[1]) / 2, 3))
            self.replace_vertex(self.prev_vertex, avg_pos)
            return avg_pos
        else:
            near_v = self.find_vertex(pos)
            if near_v == None:
                self.a_list[self.prev_vertex].add(pos)
                self.a_list[pos] = {self.prev_vertex}
                self.marks[pos] = []
                for i in range(len(link_angles)):
                    self.marks[pos].append(0)
                return pos
            else:
                avg_pos = (round((near_v[0] + pos[0]) / 2, 3), round((near_v[1] + pos[1]) / 2, 3))
                self.replace_vertex(near_v, avg_pos)
                self.a_list[self.prev_vertex].add(avg_pos)
                self.a_list[avg_pos].add(self.prev_vertex)
                return avg_pos
    
    # Find the entry link to a vertex, and apply an entry mark.
    def entry_mark(self, pos, link_angles):
        marks = self.marks[pos]
        if len(marks) != len(link_angles):
            # raise Exception('Current number of links (' + str(len(link_angles)) + ') does not match previously found number of links (' + str(len(marks)) + ').')
            print('!!! Warning: current number of links (' + str(len(link_angles)) + ') does not match previously found number of links (' + str(len(marks)) + ').')
            # In this situation, just reset the links at this vertex.
            self.marks[pos] = []
            for i in range(len(link_angles)):
                self.marks[pos].append(0)
        if self.prev_vertex == None: # If we are at the start.
            entry_angle = link_angles[0] # Assume we entered from a valid link, doesn't matter which one.
            entry_link = 0
        elif math.dist(pos, self.prev_vertex) <= self.MIN_DIST:
            return self.prev_entry_link
        else:
            # diff = [pos[i] - self.prev_vertex[i] for i in range(2)]
            diff = (pos[0] - self.prev_vertex[0], self.prev_vertex[1] - pos[1])
            entry_angle = (90 - math.degrees(math.atan2(-diff[1], -diff[0]))) % 360
            # Find the link corresponding to the entry angle.
            entry_link = 0
            for i in range(len(link_angles)):
                if self.mod_diff(link_angles[i], entry_angle, 360) < self.mod_diff(link_angles[entry_link], entry_angle, 360):
                    entry_link = i
            self.marks[pos][entry_link] += 1 # Apply entry mark.
        self.prev_entry_link = entry_link
        return entry_link

    # # Navigate during the discovery phase. Also apply an exit mark.
    # def discovery_navigate(self, pos, link_angles, entry_link):
    #     marks = self.marks[pos]
    #     target_link = None
    #     others_unmarked = True # Are all the other entrances unmarked?
    #     candidate = None # A possible direciton if all other entrances are unmarked.
    #     for i in range(len(link_angles)):
    #         if i == entry_link:
    #             continue
    #         elif candidate == None and marks[i] == 0:
    #             candidate = i
    #         elif marks[i] != 0:
    #             others_unmarked = False
    #             break
    #     if others_unmarked and len(link_angles) > 1:
    #         # We can only choose another entrance if there is another entrance to choose from...
    #         target_link = candidate
    #     elif marks[entry_link] < 2:
    #         target_link = entry_link # Go back the way we came.
    #     else:
    #         min_link = None
    #         min_val = math.inf
    #         for i in range(len(link_angles)):
    #             if marks[i] < min_val:
    #                 min_link = i
    #                 min_val = marks[i]
    #         target_link = min_link
    #     marks[target_link] += 1 # Apply exit mark.
    #     return link_angles[target_link]

    # Navigate during the discovery phase. Also apply an exit mark.
    def discovery_navigate(self, pos, link_angles, entry_link):
        marks = self.marks[pos]
        target_link = None
        candidate = None
        for i in range(len(link_angles)):
            if marks[i] == 0:
                candidate = i
                break
        if candidate == None:
            tree, path = self.dijkstra(pos)
            min_dist = math.inf
            min_pos = None
            for v in self.a_list:
                c_flag = True
                for mark in self.marks[v]:
                    if mark == 0:
                        c_flag = False
                        break
                if c_flag:
                    continue
                if tree[v] < min_dist:
                    min_dist = tree[v]
                    min_pos = v
            print(self.marks)
            print(pos, min_pos)
            tree, path = self.dijkstra(pos, min_pos)
            target_pos = path[1]
            diff = (target_pos[0] - pos[0], pos[1] - target_pos[1])
            target_angle = (90 - math.degrees(math.atan2(diff[1], diff[0]))) % 360
            target_link = 0
            for i in range(len(link_angles)):
                if self.mod_diff(link_angles[i], target_angle, 360) < self.mod_diff(link_angles[target_link], target_angle, 360):
                    target_link = i
            self.marks[pos][target_link] += 1 # Apply exit mark.
            return (90 - math.degrees(math.atan2(diff[1], diff[0]))) % 360
        else:
            self.marks[pos][candidate] += 1 # Apply exit mark.
            return link_angles[candidate]
        
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
        for vertex in self.a_list:
            for mark in self.marks[vertex]:
                if mark == 0 and dist_tree[vertex] + math.dist(vertex, self.end) < dist_tree[self.end]:
                    # If there could exist a shorter path to the end via this vertex.
                    return False
        print('Sufficient portion of maze discovered!')
        return True