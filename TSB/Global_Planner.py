#!/usr/bin/env python2
# Author: Miguel Sol
# Created. 01 October 2022
# This program creates a graph given a matrix n*or n*n
#
#
#
from collections import deque
import networkx as nx
import math

# import rospy
# from geometry_msgs.msg import Pose, PoseStamped
# from std_msgs.msg import Header
# from nav_msgs.msg import Path
import pickle

# base coordinate is the coordinate corresponding to (0,0),(y,x)
baseCoordinate = (294, 1706)
# read the gpickle file with the graph already created
# G = nx.read_gpickle("map.gpickle")
with open("MapTest", "rb") as mp:
    map = pickle.load(mp)
with open("Matrix", "rb") as mt:
    matrix = pickle.load(mt)


###################################################################################################
###################################################################################################
# implement an A* alghorithm to calculate the sortest path between two points using a graph

# this class it to give a node its parent, position and weights
class Node:
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None, g=None):
        self.parent = parent
        self.position = position

        self.g = g
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
    start_node = Node(None, start, 0)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end, None)
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
            new_node = Node(current_node, adjacent_position[1], adjacent_position[1][2])
            # Append
            children.append(new_node)
        # Loop through children
        for child in children:

            # Create the f, g, and h values
            child.g += current_node.g
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
#                   map: need to implement a matriz map with all points
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
    x = int(round(baseCoordinate[1] - coordinate[0] * 30.88))
    y = int(round(coordinate[1] * 26.04 + baseCoordinate[0]))
    return (y, x)


def conversor_pixel_to_mapa(img_x_min,img_x_max,img_y_min,img_y_max,x,y,size_of_square):
    x = math.floor((x - img_x_min) / size_of_square) if (x - img_x_min) // size_of_square != 0 else math.floor((x - img_x_min) / size_of_square) - 1
    y = math.floor((y - img_y_min) / size_of_square) if (y - img_y_min) // size_of_square != 0 else math.floor((y - img_y_min) / size_of_square) - 1
    return (x,y)


#######################################################################################################
# Path -> this function is the main function of the program and its an example of how to use
#######################################################################################################


def path():

    return getPath(
        convertToRealCoordinates((1, 22, 1)),
        convertToRealCoordinates((31, 3, 1)),
    )


######################################################################
#
# publish_Path - receives a list of the optimal path and returns it
#                in the form of a Path()
#
######################################################################


def publish_Path(path):
    msg = Path()
    msg.header.frame_id = "/map"
    msg.header.stamp = rospy.Time.now()
    if path != None:
        for wp in path:
            pose = PoseStamped()
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            msg.poses.append(pose)
    return msg


def ros(Path):
    rospy.init_node("Global_Planner")
    pub = rospy.Publisher("/path", Path, queue_size=1)
    msg = publish_Path(path())
    pub.publish(msg)

def is_land(x, y):
    return True if matrix[x][y][2] == 0 else False

if __name__ == "__main__":
    img_x_min = 268
    img_y_min = 251
    img_x_max = 4682
    img_y_max = 3321
    size_of_square = 11
    x= 2265
    y= 2432
    #print(conversor_pixel_to_mapa(img_x_min, img_x_max, img_y_min, img_y_max, x, y, size_of_square))
    #print(seeMap(path()))
    print(is_land(181,198))
