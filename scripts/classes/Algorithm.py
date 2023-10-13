from Constants import *
from classes.PathNode import PathNode
class Algorithm:

    def make_path(self, final_dijkstra_node):
        new_path = []
        count = 0
        next_node = None
        while final_dijkstra_node:
            if count == 0:
                angle = -1
                for key, value in final_dijkstra_node.neighbours.items():
                    if value == FREE:
                        angle = key
                assert angle != -1, "No neighbour with FREE value found"
                new_path.append(PathNode(final_dijkstra_node, angle))
                count += 1
            else:
                angle = -1
                for key, value in final_dijkstra_node.neighbours.items():
                    if value == next_node:
                        angle = key
                assert angle != -1, "No neighbour with given node value found"
                new_path.append(PathNode(final_dijkstra_node, angle))
                count += 1
            next_node = final_dijkstra_node
            final_dijkstra_node = final_dijkstra_node.previous
        return list(reversed(new_path))
    
    def find_min(self, node_list, closed):
        min_dist = float("inf")
        min_index = -1
        for index in range(node_list.size()):
            if node_list.nodes[index].distance < min_dist \
                and node_list.nodes[index] not in closed:
                min_dist = node_list.nodes[index].distance
                min_index = index
        return min_index

    def make_path_back(self, final_dijkstra_node):
        new_path = []
        count = 0
        next_node = None
        while final_dijkstra_node:
            if count == 0:
                angle = -1
                for key, value in final_dijkstra_node.neighbours.items():
                    if value != NEVER:
                        if value == WALLS or value == FREE:
                            angle = key
                            final_dijkstra_node.neighbours[key] = NEVER
                assert angle != -1, "No neighbour with FREE value found"
                new_path.append(PathNode(final_dijkstra_node, angle))
                count += 1
            else:
                angle = -1
                for key, value in final_dijkstra_node.neighbours.items():
                    if value == next_node:
                        angle = key
                assert angle != -1, "No neighbour with given node value found"
                new_path.append(PathNode(final_dijkstra_node, angle))
                count += 1
            next_node = final_dijkstra_node
            final_dijkstra_node = final_dijkstra_node.previous
        return list(reversed(new_path))
    
    def find_min_back(self, node_list):
        min_dist = float("inf")
        min_index = -1
        for index in range(node_list.size()):
            if node_list.nodes[index].distance < min_dist :
                min_dist = node_list.nodes[index].distance
                min_index = index
        return min_index


    def dijkstra(self, start_node, all_nodes):
        N = all_nodes.size()
        closed = set()
        start_node.distance = 0
        path = None
        paths = None
        for _ in range(N):
            if closed:
                min_index = self.find_min(all_nodes,closed)
                current_node = all_nodes.nodes[min_index]
                if current_node not in closed:
                    closed.add(current_node)
            else:
                min_index = self.find_min_back(all_nodes)
                current_node = all_nodes.nodes[min_index]
                closed.add(current_node)

            if current_node.has_free():
                path = self.make_path(current_node)
                self.cleanup(all_nodes)
                return path
                        
        if path == None:
            closed.add(start_node)
            paths = self.make_path_back(start_node)
            self.cleanup(all_nodes)
            return paths

    def cleanup(self, all_nodes):
        for node in all_nodes.nodes:
            node.distance = float("inf")
            node.previous = None