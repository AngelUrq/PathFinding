import networkx as nx
import matplotlib.pylab as plt
import numpy as np
import pickle
import time

# Definition class node
class Node:

    def __init__(self, coordinates, neighbors):
        self.coordinates = coordinates
        
        self.neighbors = neighbors
        
        self.f = 0
        self.g = 0

        self.cameFrom = None

list_nodes = []
path_list = []
start_node = None
goal_node = None

def load_data():
    pickle_in = open("Midterm1.pickle","rb")
    data = pickle.load(pickle_in)
    return data[0], data[2]

def initialize(G, pos, start, goal):
    color_map = []
    node_size = []

    #Save start and goal in Node object
    start_node = Node(start, get_neighbors(start, list(G.edges)))
    goal_node = Node(goal, get_neighbors(goal, list(G.edges)))

    for node in G:
        neighbors = get_neighbors(node, list(G.edges))
        
        list_nodes.append(Node(node,neighbors))

    #Get optimal path and cost
    path_list, cost = search_path(G,start_node,goal_node)
    path_list_coordinates = []

    #Get coordinates from Node objects
    if(path_list):   
        path_list_coordinates = get_coordinates(path_list)
        plt.title("Total cost: " + str(cost))
    else:
        plt.title("No solution was found")
  
    for node in G:
        if node == start:
            color_map.append('green')
            node_size.append(200)
        elif node == goal:
            color_map.append('blue')
            node_size.append(200)
        elif node in path_list_coordinates:
            color_map.append('purple')
            node_size.append(50)
        else:
            color_map.append('orange')
            node_size.append(50)

    # plot graph
    nx.draw_networkx(G, pos=pos, with_labels=False, node_color=color_map, node_size=node_size)
    plt.xticks(np.arange(0, 20))
    plt.yticks(np.arange(0, 20))
    plt.show()

#Manhattan distance between a and b
def heuristic(a,b):
    return (abs(a[0]-b[0]) + abs(a[1]-b[1]))*1

#Get all node neighbors, avoiding the neighbor from above
def get_neighbors(node, edges):
    neighbors = []
    
    for edge in edges:
        if(node in edge):
            for relationed_node in edge:
                if(relationed_node != node and relationed_node[1] <= node[1]):
                    neighbors.append(relationed_node)
            
    return neighbors

#Get minimun value of f(n) from list_nodes
def select_min_f(list_nodes):
    min_node = list_nodes[0]

    for node in list_nodes:
        if node.f < min_node.f:
            min_node = node

    return min_node
        

#Get optimal path and cost from where came the node    
def reconstruct_path(node):
    path = []
    cost = node.g

    while node != None:
        path.append(node)
        node = node.cameFrom

    path.reverse()
    
    return path, cost

#Find index of the neighbor in the list of nodes, checking its coordinates
def find_index_neighbor_in_list(neighbor):
    for i in range(len(list_nodes)):
        if list_nodes[i].coordinates == neighbor:
            return i

    return -1

#A* implementation
def search_path(G, start_node, goal_node):
    openSet = [start_node]
    closedSet = []

    while openSet:
        current = select_min_f(openSet)

        if current.coordinates == goal_node.coordinates:
            return reconstruct_path(current)

        openSet.remove(current)
        closedSet.append(current)

        for neighbor in current.neighbors:
            #in order to modify original list_nodes, we need the index
            #list_nodes[i] is the neighbor Node object in current.neighbors
            #because current.neighbors are only coordinates

            i = find_index_neighbor_in_list(neighbor)
            
            if list_nodes[i] in closedSet:
                continue
                    
            tentative_g = current.g + 1

            if list_nodes[i] not in openSet:
                openSet.append(list_nodes[i])
            elif tentative_g >= list_nodes[i].g:
                continue
                    
            list_nodes[i].cameFrom = current
            list_nodes[i].g = tentative_g
            list_nodes[i].f = list_nodes[i].g + heuristic(list_nodes[i].coordinates, goal_node.coordinates)

    print "No solution found"
    return None, 0

def get_coordinates(list_nodes_path):
    coordinates_path = []

    for node in list_nodes_path:
        coordinates_path.append(node.coordinates)
    
    return coordinates_path

def main():
    G, pos = load_data()
    start = (2,19)
    goal = (17,0)
    initialize(G, pos, start, goal)

if __name__ == "__main__":
    main()
