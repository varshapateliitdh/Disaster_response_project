import math
import numpy as np
from Constants import *
import matplotlib.pyplot as plt


class NodeList:

    def __init__(self, node_list=[]):
        self.nodes = []
        self.nodes_dict = {}
        for node in node_list:
            self.add_node(node)

    def add_node(self, node):
        self.nodes.append(node)
        self.nodes_dict[node.coordinate] = len(self.nodes) - 1

    def get_index(self, node):
        if node.coordinate in self.nodes_dict:
            return self.nodes_dict[node.coordinate]
        else:
            return -1

    def get_all_nodes(self):
        return self.nodes

    def return_if_same(self, node):
        min_dist = float('inf')
        index = 0
        if len(self.nodes) == 0:
            return None
        for i in range(len(self.nodes)):
            dist = np.linalg.norm(np.array(self.nodes[i].coordinate) -
                                  np.array(node.coordinate))
            if dist < min_dist:
                min_dist = dist
                index = i
        if min_dist < DISTANCE_THRESHOLD:
            return self.nodes[index]
        return None
    


    def size(self):
        return len(self.nodes)
