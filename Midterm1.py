import networkx as nx
import matplotlib.pylab as plt
import numpy as np
import pickle

class Node:

    def __init__(self, coordinates):
        self.coordinates = coordinates

def load_data():
    # load graph information
    pickle_in = open("Midterm1.pickle","rb")
    data = pickle.load(pickle_in)
    return data[0], data[2]

def initialize(G, pos, start, goal):
    for n in G:
        print(type(n))
    # define start and goal nodes
    color_map = []
    node_size = []
    for node in enumerate(G):
        # start node
        if node[1] == start:
            color_map.append('green')
            node_size.append(200)
        # end node
        elif node[1] == goal:
            color_map.append('blue')
            node_size.append(200)
        # all others
        else:
            color_map.append('red')
            node_size.append(50)

    # plot graph
    nx.draw_networkx(G, pos=pos, with_labels=False, node_color=color_map, node_size=node_size)
    plt.xticks(np.arange(0, 20))
    plt.yticks(np.arange(0, 20))
    plt.show()



# def heuristic():
#     h = "your heuristic estimate"
#     return h


# def getNeighbors():
#     return "list of neighors"


def searchPath(G):
    pass


# def plotPath(G, path):
#     nx.draw("show the path within the graph")
#     print path


def main():
    G, pos = load_data()
    start_node = (2,19)
    goal_node = (17,0)
    initialize(G, pos, start_node, goal_node)

    ''' you have to develop the rest of the functions '''
    # searchPath()

    # plotPath()



# when you call the script, it will start here
if __name__ == "__main__":
    main()
