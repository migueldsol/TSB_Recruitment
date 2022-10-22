# Author: Miguel Sol
# Created. 01 October 2022
# This program creates a graph given a matrix n*or n*n
#
#
#
from collections import deque
import networkx as nx
from math import *
import pickle


# base coordinate is the coordinate corresponding to (0,0),(y,x)
baseCoordinate = (294, 1706)
# read the gpickle file with the graph already created
#G = nx.read_gpickle("map.gpickle")
pickle_in = open("map.pickle","rb")
G = pickle.load(pickle_in)


###################################################################################################
###################################################################################################
# implement an A* alghorithm to calculate the sortest path between two points using a graph

# this class it to give a node its parent, position and weights
class Node:
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


#######################################################################################################
# astar function -> this function receives two coordinates and returns the shortest path between them
#                   using the A* algorithm
#
#                   start: the starting coordinate
#                   end: the ending coordinate
#
#                   returns: a list of coordinates that represent the shortest path
#######################################################################################################


def astar(start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]  # Return reversed path

        # Generate children
        children = []
        for adjacent_position in G.edges(current_node.position):
            # Get node position
            # Create new node
            new_node = Node(current_node, adjacent_position[1])
            # Append
            children.append(new_node)
        # Loop through children
        for child in children:

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = sqrt(
                ((child.position[0] - end_node.position[0]) ** 2)
                + ((child.position[1] - end_node.position[1]) ** 2)
            )
            child.f = child.g + child.h

            # Child is on the closed list
            if child in closed_list:
                continue

            # Child is already in the open list
            if child in open_list:
                continue

            # Add the child to the open list
            open_list.append(child)


#######################################################################################################
# seeMap function -> this function receives a path and prints it in the console
#
#                   path: the path to be printed
#
#######################################################################################################


def seeMap(path):
    temp = ""
    for i in range(45, -1, -1):
        for n in range(50, -1, -1):
            if convertToRealCoordinates(map[n][i]) in path:
                temp += "0 "
            elif map[n][i][2] == 1:
                temp += "x "
            else:
                temp += "- "
        temp += "\n"
    print(temp)


###########################################################################################################
# getPath function -> this function returns the shortest path between two coordinates in real coordinates
#
#                   start: the starting coordinate
#                   end: the ending coordinate
#
#                   returns: a list of coordinates (real) that represent the shortest path
###########################################################################################################


def getPath(start, end):
    start = convertFromRealCoordinates(start)
    end = convertFromRealCoordinates(end)
    Path = astar(start, end)
    print(Path)
    for i in range(len(Path)):
        Path[i] = convertToRealCoordinates(Path[i])
    return Path


#######################################################################################################
# convertFromRealCoordinates function -> this function receives a coordinate in real coordinates and
#                                        returns the corresponding coordinate in the data structure
#
#                   coordinate: the coordinate to be converted
#
#                   returns: the corresponding coordinate in the data structure
#######################################################################################################


def convertFromRealCoordinates(coordinate):
    x = round((baseCoordinate[1] - coordinate[1]) // 30.88)
    y = round((coordinate[0] - baseCoordinate[0]) // 26.04)
    return (x, y, 1)


#######################################################################################################
# convertToRealCoordinates function -> this function receives a coordinate in the data structure and
#                                      returns the corresponding coordinate in real coordinates
#
#                   coordinate: the coordinate to be converted
#
#                   returns: the corresponding coordinate in real coordinates
#######################################################################################################


def convertToRealCoordinates(coordinate):
    x = round(baseCoordinate[1] - coordinate[0] * 30.88)
    y = round(coordinate[1] * 26.04 + baseCoordinate[0])
    return (y, x)


#######################################################################################################
# main function -> this function is the main function of the program and its an example of how to use
#######################################################################################################


def main():
    print(
            getPath(
                convertToRealCoordinates((1, 22, 1)),
                convertToRealCoordinates((31, 3, 1)),
            )
    )


if __name__ == "__main__":
    main()
