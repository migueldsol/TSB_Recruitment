#!/usr/bin/env python2
# Author: Miguel Sol
# Created. 01 October 2022
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
with open("MapTest.pickle", "rb") as mp:
    # with open("D:\TSB_Recruitment\TSB\MapTest.pickle", "rb") as mp:
    G = pickle.load(mp)
with open("Matrix.pickle", "rb") as mt:
    # with open("D:\TSB_Recruitment\TSB\Matrix.pickle", "rb") as mt:
    map = pickle.load(mt)

#######################################################################################################
# seeMap function -> this function receives a path and prints it in the console
#                   map: need to implement a matriz map with all points
#                   path: the path to be printed
#
#######################################################################################################


def seeMap(path, img_x_min, img_x_max, img_y_min, img_y_max, size_of_square):
    temp = ""
    for n in range((img_y_max - img_y_min) // size_of_square):
        for i in range((img_x_max - img_x_min) // size_of_square):
            if (map[i][n][0], map[i][n][1]) in path:
                temp += "0 "
            elif map[i][n][2] == 1:
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
def dist(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def getPath(start, end):
    Path = G.astar_path(G, start, end, heuristic=dist, weight="weight")
    return Path


#######################################################################################################
# convertToRealCoordinates function -> this function receives a coordinate in the data structure and
#                                      returns the corresponding coordinate in real coordinates
#
#                   coordinate: the coordinate to be converted
#
#                   returns: the corresponding coordinate in real coordinates
#######################################################################################################


def conversor_pixel_to_mapa(
    img_x_min, img_x_max, img_y_min, img_y_max, x, y, size_of_square
):
    x = (
        math.floor((x - img_x_min) / size_of_square)
        if (x - img_x_min) // size_of_square != 0
        else math.floor((x - img_x_min) / size_of_square) - 1
    )
    y = (
        math.floor((y - img_y_min) / size_of_square)
        if (y - img_y_min) // size_of_square != 0
        else math.floor((y - img_y_min) / size_of_square) - 1
    )
    return (x, y)


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
    start = conversor_pixel_to_mapa(
        img_x_min, img_x_max, img_y_min, img_y_max, 3155, 2075, size_of_square
    )
    end = conversor_pixel_to_mapa(
        img_x_min, img_x_max, img_y_min, img_y_max, 2762, 2220, size_of_square
    )
    # print(getPath(start, end))
    print(
        seeMap(
            getPath(start, end),
            img_x_min,
            img_x_max,
            img_y_min,
            img_y_max,
            size_of_square,
        )
    )
