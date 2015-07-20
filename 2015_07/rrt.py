#!/usr/bin/env python
import numpy as np
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class RRTTree:
    def __init__(self, root_pos, max_nodes_num, step, goal_pos):
        self.root_pos = root_pos
        self.max_nodes_num = max_nodes_num #How many points will be sampled
        self.step = step
        self.nodes = [self.root_pos]
        self.edges = {}
        self.goal_pos = goal_pos
        plt.ion()
        self.fig = plt.figure()
        self.ax = Axes3D(self.fig)
        self.ax.scatter([self.root_pos[0], self.goal_pos[0]],
                        [self.root_pos[1], self.goal_pos[1]],
                        [self.root_pos[2], self.goal_pos[2]],
                        c = 'r', s= 120)
        plt.draw()

    #Check All nodes to find Most Nearest to query node
    def getNearestNode(self, query_pos):
        node_id = 0
        nearest_distance = np.linalg.norm(self.nodes[0] - query_pos)
        for i, node in enumerate(self.nodes):
            tmp_distance = np.linalg.norm(node - query_pos)
            if  tmp_distance < nearest_distance:
                node_id = i
                nearest_distance = tmp_distance
        return node_id

    #Always parent_node (from) -> child_node (to)
    #So child_node(to) id is unique 
    def addEdge(self, from_index, to_index):
        self.edges[to_index] = from_index

    def addNode(self, pos):
        self.nodes.append(pos)

    # Points will be -100 < x < 100
    #                -100 < y < 100
    #                -100 < z < 100
    def createRandomPoints(self):
        return np.array([random.random() * 200 - 100,
                         random.random() * 200 - 100,
                         random.random() * 200 - 100
                         ])

    def drawRRT(self, node_ids, edge_keys):
        for node_id in node_ids:
            self.ax.scatter([self.nodes[node_id][0]],
                            [self.nodes[node_id][1]],
                            [self.nodes[node_id][2]],
                            c = 'k')
        for edge_key in edge_keys:
            from_key = self.edges[edge_key]
            self.ax.plot([self.nodes[from_key][0], self.nodes[edge_key][0]],
                         [self.nodes[from_key][1], self.nodes[edge_key][1]],
                         [self.nodes[from_key][2], self.nodes[edge_key][2]],
                         "b"
                         )
        plt.draw()

    def showPath(self):
        child_node_id = len(self.nodes) - 1
        parent_node_id = self.edges[child_node_id]

        self.ax.plot([self.goal_pos[0], self.nodes[child_node_id][0]],
                     [self.goal_pos[1], self.nodes[child_node_id][1]],
                     [self.goal_pos[2], self.nodes[child_node_id][2]],
                     "r", linewidth = 5
                     )
        plt.draw()

        while True:
            self.ax.plot([self.nodes[parent_node_id][0], self.nodes[child_node_id][0]],
                         [self.nodes[parent_node_id][1], self.nodes[child_node_id][1]],
                         [self.nodes[parent_node_id][2], self.nodes[child_node_id][2]],
                         "r", linewidth = 5
                         )
            plt.draw()
            if parent_node_id == 0:
                break
            child_node_id = parent_node_id
            parent_node_id = self.edges[child_node_id]

    def plan(self):
        while len(self.nodes) < self.max_nodes_num:
            random_sampling_pos = self.createRandomPoints()
            nearest_nodes_id = self.getNearestNode(random_sampling_pos)
            nearest_nodes_pos = self.nodes[nearest_nodes_id]
            #With random_sampling_pos and nearest_node,
            #We will generate new_pos which will be registered in self.nodes
            new_pos = nearest_nodes_pos + self.step * (random_sampling_pos - nearest_nodes_pos) / np.linalg.norm(random_sampling_pos - nearest_nodes_pos)
            self.addNode(new_pos)
            self.addEdge(nearest_nodes_id, len(self.nodes) - 1)
            if np.linalg.norm(new_pos - self.goal_pos) < self.step:
                break
            self.drawRRT([len(self.nodes) - 1], [len(self.nodes) - 1])

        if len(self.nodes) == self.max_nodes_num:
            print "Couldn't find the path..."
        else:
            print "Path was Found!!"
            self.showPath()
            raw_input("Press any Key to END")

if __name__ == "__main__":
    #Search Range (-100 ~ 100, -100 ~ 100, - 100 ~ 100)
    #root_pos : (0,0,0) goal_pos :(50, 50, 50), sample_points: 1000
    #Sample length is 20
    rrt_tree = RRTTree(np.array([0,0,0]), 1000, 20, np.array([50, 50, 50]))
    raw_input("Press any Key to START")
    rrt_tree.plan()
