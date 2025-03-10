# Recruitment _Project_ Tecnico Solar Boat

Development of a global planer and a program that converts a png image into a data structure for TSB_Recruitment 2022/2023

## Electric Systems - Global Planner

## Objective

Create, in ros, a global planner node that is able to trace the shortest path between two given coordinates and publish it to a topic. Also it was given a nautical Chart with the purpose of transforming it into a data structure for the global planner.

- The Nautical Chart

![](/TSB/CartaNautica.png)

- Example of Data Structure (this is the 2d list that will be converted into a graph)

![](/TSB/DataStructure.png)

## Data Structure

I've decided to use a graph composed by only the nodes that are navigatable. Used the python networkx library

**Node Representation**

- it is a tupple ( x , y , 0 or 1), being x and y data coordinates, 0 representing land and 1 representing water

**Edge Representation**

- an edge is a tupple with the start node, the neighbour node, and the weight of the connection

## A\* Algorithm

Today, one of the most used algorithms is the A\* so i ended using it, i followed GeeksForGeeks guide to write this algorithm and used Euclidean Distance to calculate the heuristic of each node

## Converting the map to a structure

- I used the python pillow library to get the pixel colors and paint to get pixel coordinates

## Learning ROS

- Being a ros an OS that i never used i started to read the oficial documentation just to get a grasp of its utility. In the end I learned the basics needed to create a node and publish to a topic with the youtube [crash course](https://www.youtube.com/watch?v=wfDJAYTMTdk) by Robotics Back-End

## Using ROS, VMWARE and Unity

- TSB provided us with an instance of a VM already with ROS melodic instaled and a complete guide of how to connect it to unity. Also the provided us with the link to the [Njord Competition guide](https://njord.gitbook.io/digital-competition-2022/) which was very helpfull to install Unity

## Results

- Was able to:

  - create a map convertor to graph [here](TSB/ConvertPNGtoGraph.py)

  - create an A\* algorithm aplied to the graph(Global Planner) [here](TSB/GlobalPlanner.py)

  - create a ros global_planner package with a node that publish a path to a /path topic [here](TSB/Global_Planner.py)

- This results only apply to this image of the chart with the objective of implementing it to the whole nautical chart.

![](CartaNauticaReduzida.png)

## Installing the ros global_planner package

- Inside catkin_ws/src/ run this command that will create a new package with the name global_planner

  ```bash
  catkin_create_pkg global_planner rospy nav_msgs geometry_msgs std_msgs
  ```

- After this enter the new folder and create a /scripts folder

- Inside the /scripts folder insert the Global_Planner.py [here](/TSB/Global_Planner.py) and graph.py [here](/TSB/unusedcode/graph.py)

- First run the graph.py and then after starting the roscore run the Global_Planner.py to start publishing the path to the /Path topic
