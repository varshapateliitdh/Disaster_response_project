import os
import rospy
import numpy as np
from classes.Node import Node
from Constants import *
from classes.Timer import Timer
from classes.NodeList import NodeList
from classes.FixedList import FixedList
from classes.Algorithm import Algorithm


class Controller:

    def __init__(self, pose, movement):
        self.pose = pose
        self.home = os.path.expanduser('~')
        self.rgb_images = FixedList(MAX_IMAGES)
        self.movement = movement
        self.all_nodes = []
        self.node_list = NodeList()
        self.prev_node = None
        self.current_node = None
        self.timer = Timer()
        self.analysed_angles = []
        self.next_destination = None
        self.temp_walls = 0
        self.counter = 0
        self.closed = []

    
    def obstacle(self, threshold):
        if self.temp_walls == 0:
            rgb_images = self.rgb_images[9][165:195]
            for i in range(9):
                rgb_images += self.rgb_images[i][165:195]
        elif self.temp_walls == 1:
            rgb_images = self.rgb_images[9][255:285]
            for i in range(9):
                rgb_images += self.rgb_images[i][255:285]
        elif self.temp_walls == 2:
            rgb_images = self.rgb_images[9][345:360]
            rgb_images += self.rgb_images[9][0:15]
            for i in range(9):
                rgb_images += self.rgb_images[i][345:360]
                rgb_images += self.rgb_images[i][0:15]
        elif self.temp_walls == 3:
            rgb_images = self.rgb_images[9][75:105]
            for i in range(9):
                rgb_images += self.rgb_images[i][75:105]
        mean_distance = np.mean(rgb_images)
        print("Mean Distance to Wall:", mean_distance)
        return mean_distance <= threshold

    
    def skewness(self, arr):
        median = sorted(arr)[len(arr)//2]
        higher_values = []
        for i, num in enumerate(arr):
            if num > median:
                higher_values.append(i - len(arr)//2)
        sum_distances = sum(higher_values)
        return 4*sum_distances/np.mean(arr)
    
    def skewness2(self, arr):
        max_value = max(arr)
        differences = [abs(max_value - num) for num in arr]
        significance = [diff / max_value * 100 for diff in differences]
        index = np.where(arr == max_value)[0]
        return ((index - COL_SPLITS//2) * np.mean(significance))[0]/10
    
    def lidar_callback(self, msg):
       try:
           ranges = msg.ranges      
           self.rgb_images.fixedAppend(ranges)
       except Exception as e:
           rospy.logerr(e)


    def initialize(self):
        x, y = self.pose.pose.position.x, self.pose.pose.position.y
        new_node = Node((x, y))
        temp_node = self.node_list.return_if_same(new_node)
        if temp_node:
            new_node = temp_node
        else:
            self.node_list.add_node(new_node)
        self.current_node = new_node
        self.movement.state = ANALYZING
        self.timer.set_timeout(timeout_s=10)

    def analyze_surroundings(self):
        if self.current_node.coordinate in self.closed:
            index = self.closed.index(self.current_node.coordinate)
            self.current_node = self.node_list.nodes[index]
            self.movement.state = THINKING
            self.prev_node = self.current_node
            self.current_node = None

        else:
            current_angle = self.movement.current_angle
            print("Node Children: ", self.current_node)
            if self.temp_walls < 4:
                if self.obstacle(ANA_OBSTACLE_THRESHOLD):
                    self.current_node.neighbours[self.movement.current_angle] = WALL
                else:
                    if self.current_node.neighbours[self.movement.current_angle]!= NEVER:
                        self.current_node.neighbours[self.movement.current_angle] = FREE

                self.movement.current_angle +=90
                if self.movement.current_angle == 360:
                    self.movement.current_angle = 0
                self.temp_walls += 1

            elif self.temp_walls == 4:
                if self.prev_node:
                    if self.current_node.coordinate not in self.closed:
                        self.current_node.neighbours[
                            (current_angle+180) % 360] = WALLS
                    
                self.temp_walls = 0
                self.movement.state = THINKING
                self.prev_node = self.current_node
                self.current_node = None

    def path_finder(self):
        algorithm = Algorithm()
        self.closed = []
        path_list= algorithm.dijkstra(self.prev_node, self.node_list)

        if path_list == None or len(path_list)==0:
            raise Exception("No path found, No more paths to explore!!")
        
        for nodez in self.node_list.nodes:
            self.closed.append(nodez.coordinate)
        self.closed.pop()
        print("-------------PATH FOUND-----------")
        if path_list:
            for path in path_list:
                print(path, end=" ")
            print()
            print("-----------------------------")
            self.movement.path = path_list
            self.movement.state = PATH_TRAVERSE


    def update(self):
        if not self.timer.check_timeout():
            return
        if self.movement.state == FREE_ROAM and self.obstacle(MOV_OBSTACLE_THRESHOLD):
            self.movement.state = INIT

        if self.movement.state == INIT:
            self.initialize()

        elif self.movement.state == ANALYZING:
            self.analyze_surroundings()

        elif self.movement.state == THINKING:
            self.path_finder()