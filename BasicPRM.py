import matplotlib.pyplot as plt

import networkx as nx
import random
import numpy as np
import math

# sort nearest neighbour brute force
import heapq

from scipy.spatial.distance import euclidean

from collisionchecker import CollisionChecker
import testsuites


def nearestNeighboursX(pos, graph, radius):
    """ Brute Force method to find all nodes of a 
    graph near the given position **pos** with in the distance of
    **radius** in **increasing order**"""
    
    heap = list()
    for node in graph.nodes(data=True): # using (data=True) will generate a list of nodes with all attributes
        if euclidean(node[1]['pos'],pos) < radius:
            # use a heap-queue to sort the nodes in increasing order
            heapq.heappush(heap, (euclidean(node[1]['pos'] ,pos), node))

    result = list()
    while len(heap) > 0 :
         result.append(heapq.heappop(heap)) 
    
    return result
    
    
def inSameConnectedComponent(node1, node2, graph):
    """ Check whether to nodes are part of the same connected component using
        functionality from NetworkX
    """
    for connectedComponent in nx.connected_components(graph):
        if (node1 in connectedComponent) & (node2 in connectedComponent):
            return True
        
    return False


def getRandomFreePosition(collChecker):
    limits = collChecker.getEnvironmentLimits()        
    pos = [random.uniform(limit[0],limit[1]) for limit in limits]
    while collChecker.pointInCollision(pos):
        pos = [random.uniform(limit[0],limit[1]) for limit in limits]
    return pos


class BasicPRMPlanner:
    def __init__(self, scene, limits):
        self.graph = nx.Graph()
        self.collisionChecker = CollisionChecker(scene, limits)

    def learn(self, radius, numNodes):
        i = 0
        while i < numNodes:
        
            # Generate a randomly chosen, free configuration
            pos = getRandomFreePosition(self.collisionChecker)
            
            # Find set of candidates to connect to sorted by distance
            result = nearestNeighboursX(pos, self.graph, radius)
        
            # check connection
            self.graph.add_node(i, pos=pos)
            for idx, data in enumerate(result):
                if not inSameConnectedComponent(i, data[1][0], self.graph):
                    if not self.collisionChecker.lineInCollision(pos,data[1][1]['pos']):
                        self.graph.add_edge(i,data[1][0])
                    
            i += 1
        
        
    def findPath(self, start, goal, radius):
        # find nearest, collision-free connection between node on graph and start
        result = nearestNeighboursX(start, self.graph, radius)
        for node in result:
            if not self.collisionChecker.lineInCollision(start,node[1][1]['pos']):
                self.graph.add_node("start", pos=start)
                self.graph.add_edge("start", node[1][0])
                break
                
        # find nearest, collision-free connection between node on graph and goal
        result = nearestNeighboursX(goal, self.graph, radius)
    
        for node in result:
            if not self.collisionChecker.lineInCollision(goal,node[1][1]['pos']):
                self.graph.add_node("goal", pos=goal)
                self.graph.add_edge("goal",node[1][0])
                break
                
        # find shortest path on graph
        path = nx.shortest_path(self.graph, "start", "goal")
        
        # return nodelist
        return path
    
    
    def visualize(self, solution, nodeSize=300.0, ax=None):
        # get a list of posiations of all nodes by returning the content of the attribute 'pos'
        pos = nx.get_node_attributes(self.graph, 'pos')
    
        # draw graph (nodes colorized by degree)
        nx.draw_networkx_nodes(self.graph, pos,  cmap=plt.cm.Blues, ax = ax, node_size=nodeSize)
        nx.draw_networkx_edges(self.graph, pos, ax = ax)
        Gcc = sorted(nx.connected_components(self.graph), key=len, reverse=True)
        G0 = self.graph.subgraph(Gcc[0])# = largest connected component

        # how largest connected component
        nx.draw_networkx_edges(G0,pos,
                               edge_color='b',
                               width=3.0, ax = ax
                               )

        self.collisionChecker.drawObstacles(ax)
    
    
        # draw nodes based on solution path
        Gsp = nx.subgraph(self.graph, solution)
        nx.draw_networkx_nodes(Gsp, pos,
                                node_size=300,
                                node_color='g',  ax = ax)
        
        # draw edges based on solution path
        nx.draw_networkx_edges(Gsp, pos,alpha=0.8, edge_color='g', width=10, ax = ax)
        
        # draw start and goal
        if "start" in self.graph.nodes(): 
            nx.draw_networkx_nodes(self.graph,pos,nodelist=["start"],
                                       node_size=300,
                                       node_color='#00dd00',  ax = ax)
            nx.draw_networkx_labels(self.graph,pos,labels={"start": "S"},  ax = ax)


        if "goal" in self.graph.nodes():
            nx.draw_networkx_nodes(self.graph, pos, nodelist=["goal"],
                                       node_size=300,
                                       node_color='#DD0000',  ax = ax)
            nx.draw_networkx_labels(self.graph, pos, labels={"goal": "G"},  ax = ax)
    
