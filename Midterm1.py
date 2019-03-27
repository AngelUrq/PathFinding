import networkx as nx
import matplotlib.pylab as plt
import numpy as np
import pickle

# Definition class node
class Node:

    def __init__(self, coordinates, neighbors):
        self.coordinates = coordinates
        
        self.neighbors = neighbors
        
        self.f = 0
        self.g = 0
        self.h = 0

        self.cameFrom = None

list_nodes = []
start_node = None
goal_node = None

def load_data():
    # load graph information
    pickle_in = open("Midterm1.pickle","rb")
    data = pickle.load(pickle_in)
    return data[0], data[2]

def initialize(G, pos, start, goal):

    # define start and goal nodes
    color_map = []
    node_size = []

    start_node = Node(start, get_neighbors(start, list(G.edges)))
    goal_node = Node(goal, get_neighbors(goal, list(G.edges)))

    for node in G:
        neighbors = get_neighbors(node, list(G.edges))

        list_nodes.append(Node(node,neighbors))

        # start node
        if node == start:
            color_map.append('green')
            node_size.append(200)
        # end node
        elif node == goal:
            color_map.append('blue')
            node_size.append(200)
        # all others
        else:
            color_map.append('red')
            node_size.append(50)


    search_path(G,start_node,goal_node)

    # plot graph
    nx.draw_networkx(G, pos=pos, with_labels=False, node_color=color_map, node_size=node_size)
    plt.xticks(np.arange(0, 20))
    plt.yticks(np.arange(0, 20))
    plt.show()

def heuristic(a,b):
    return np.sqrt((a[0]+b[0])**2+(a[1]+b[1]**2))


def get_neighbors(node, edges):
    neighbors = []
    
    for edge in edges:
        if(node in edge):
            for relationed_node in edge:
                if(relationed_node != node):
                    neighbors.append(relationed_node)
            
    return neighbors

def select_min_f(list_nodes):
    min_node = list_nodes[0]

    for node in list_nodes:
        if node.f < min_node.f:
            min_node = node

    return min_node
        
        
def reconstruct_path(node):
    i = 1
    while node != None:
        if(node.cameFrom != None):
            x2 = node.coordinates[0]
            y2 = node.coordinates[1]

            x1 = node.cameFrom.coordinates[0]
            y1 = node.cameFrom.coordinates[1]

            if(x1 - x2 < 0):
                print(str(i) + ". Derecha")
            elif(x1 - x2 > 0):
                print(str(i) + ". Izquierda")

            if(y1 - y2 < 0):
                print(str(i) + ". Arriba")
            elif(y1 - y2 > 0):
                print(str(i) + ". Abajo") 

            i = i + 1  

        node = node.cameFrom

def find_index_neighbor_in_list(neighbor):
    for i in range(len(list_nodes)):
        if list_nodes[i].coordinates == neighbor:
            return i

    return -1

def search_path(G, start_node, goal_node):
    openSet = [start_node]
    closedSet = []

    while openSet:
        current = select_min_f(openSet)

        if current.coordinates == goal_node.coordinates:
            reconstruct_path(current)
            break
        
        openSet.remove(current)
        closedSet.append(current)

        for neighbor in current.neighbors:
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
    

# def plotPath(G, path):
#     nx.draw("show the path within the graph")
#     print path


def main():
    G, pos = load_data()
    start = (2,19)
    goal = (17,0)
    initialize(G, pos, start, goal)


    ''' you have to develop the rest of the functions '''
    # searchPath()

    # plotPath()


# when you call the script, it will start here
if __name__ == "__main__":
    main()
