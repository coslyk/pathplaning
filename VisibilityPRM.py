import matplotlib.pyplot as plt

import networkx as nx
import random

# sort nearest neighbour brute force
import heapq

from scipy.spatial.distance import euclidean

from collisionchecker import CollisionChecker


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


def getRandomFreePosition(collChecker):
    limits = collChecker.getEnvironmentLimits()        
    pos = [random.uniform(limit[0],limit[1]) for limit in limits]
    while collChecker.pointInCollision(pos):
        pos = [random.uniform(limit[0],limit[1]) for limit in limits]
    return pos


class VisibilityPRMPlanner:
    def __init__(self, scene, limits):
        self.graph = nx.Graph()
        self.collisionChecker = CollisionChecker(scene, limits)

    def learn(self, numTests):
        ntry = 0
        i = 0
        guards = []
        connections = []
        while ntry < numTests:
        
            # Generate a randomly chosen, free configuration
            pos = getRandomFreePosition(self.collisionChecker)
            visibleNode = None
            visibleGuard = None

            # Check if pos is a connection node or a guard node
            for guard in guards:
                connectionFound = False
                nodeDatas = self.graph.nodes(data=True) # Generate a list of nodes with all attributes
                for guardNode in guard:
                    # pos is visible to the guard?
                    if not self.collisionChecker.lineInCollision(pos, nodeDatas[guardNode]['pos']):

                        # First visible guard?
                        if visibleNode is None:
                            visibleNode = guardNode
                            visibleGuard = guard
                        
                        # Second visible guard => pos is connection node
                        else:
                            connectionFound = True
                            connections.append(i)

                            # Connect two guards in graph
                            self.graph.add_node(i, pos=pos)
                            self.graph.add_edge(i, visibleNode)
                            self.graph.add_edge(i, guardNode)

                            # Merge two guards
                            guard += visibleGuard
                            guards.remove(visibleGuard)
                            i += 1

                        # Search next guard
                        break
                    
                if connectionFound:
                    break
            
            # No visible node => guard node
            if visibleNode is None:
                self.graph.add_node(i, pos=pos)
                guards.append([i])
                i += 1

            ntry += 1
        
        
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
    
