# Author: Miguel Sol
# Created. 01 October 2022
# This program creates a graph given a matrix n*or n*n
#
#
#
from collections import deque
import networkx as nx
from math import *
from tsb2 import *

# G = nx.read_gexf("/home/migueldsol/repositories/Private/TSB/ola.gexf")
G = nx.read_gpickle("map.gpickle")
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


def seeMap(path):
    temp = ""
    # iterate column
    for y in range(0, len(matrix[0])):
        # iterate line
        for x in range(0, len(matrix)):
            if matrix[x][y][2] == 1:
                temp += "- "
            else:
                temp += "0 "
        temp += "\n"
    # open text file
    text_file = open("data.txt", "w")

    # write string to file
    text_file.write(temp)

    # close file
    text_file.close()


def getPath(start, end):
    start = convertFromRealCoordinates(start)
    end = convertFromRealCoordinates(end)
    Path = astar(start, end)
    print(Path)
    for i in range(len(Path)):
        Path[i] = convertToRealCoordinates(Path[i])
    return Path


"""
# base coordinate is the coordinate corresponding to (0,0),(y,x)
baseCoordinate = (294, 1706)

# this function receives a coordinate where the boat can be and converts it into the data structure
def convertFromRealCoordinates(coordinate):
    x = round((baseCoordinate[1] - coordinate[1]) // 30.88)
    y = round((coordinate[0] - baseCoordinate[0]) // 26.04)
    return (x, y, 1)


# this function transforms a coordinate froma the data structure into a real one
def convertToRealCoordinates(coordinate):
    x = round(baseCoordinate[1] - coordinate[0] * 30.88)
    y = round(coordinate[1] * 26.04 + baseCoordinate[0])
    return (y, x)
"""


def main():
    print(
        seeMap(
            getPath(
                convertToRealCoordinates((1, 22, 1)),
                convertToRealCoordinates((31, 3, 1)),
            )
        )
    )
    # print(convertToRealCoordinates((50, 45, 1)))
    # function to see map with x (where you can navigate) and - (land)
    # Path = astar((1, 22, 1), (50, 45, 1))


# print(Path)


if __name__ == "__main__":
    main()
