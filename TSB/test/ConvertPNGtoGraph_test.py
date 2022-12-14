import sys
from PIL import Image
import networkx as nx
from math import sqrt
import pickle

############################################################################################
# matrix_to_txtFile -> this function will create a txt file with the 0 (land) and -(water)
#                     matrix: matrix to be written in the txt file
############################################################################################
def matrix_to_txtFile(matrix):
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


############################################################################################
# get_matrix -> this function will pick a png image will get the mode of a n*n square
#               of pixels and will create 2d matrix of the image with each square
#               being evaluated by a tupple (x,y,(1 or 0)), depending if its land or water
#  img_x_min: x coordinate of the top left corner of the image
#  img_y_min: y coordinate of the top left corner of the image
#  img_x_max: x coordinate of the bottom right corner of the image
#  img_y_max: y coordinate of the bottom right corner of the image
#  image: name of png file in directory
#  size_of_square: choose the size of each square you want to evaluate in pixels
#
############################################################################################


def get_matrix(img_x_min, img_y_min, img_x_max, img_y_max, image, size_of_square):
    img = Image.open(image)
    matrix = []
    for x in range(0, (img_x_max - img_x_min) // size_of_square):
        matrix.append([])
        for y in range(0, ((img_y_max - img_y_min) // size_of_square)):
            # matrix of all colors in the 11x11 pixel squares
            temp_color_matrix = []
            # cicle to get all colors in the 11x11 pixel squares
            for l in range(0, size_of_square):
                for k in range(0, size_of_square):
                    temp_color_matrix.append(
                        img.getpixel(
                            (
                                x * size_of_square + img_x_min + l,
                                y * size_of_square + img_y_min + k,
                            )
                        )
                    )
            # get mode of temp_color_matrix
            temp_color = max(set(temp_color_matrix), key=temp_color_matrix.count)
            # if the color is above this rgb value intervale is considered water
            if temp_color[0] > 190 and temp_color[1] > 200 and temp_color[2] > 200:
                matrix[x].append(
                    (
                        x,
                        y,
                        1,
                    )
                )
            else:
                matrix[x].append(
                    (
                        x,
                        y,
                        0,
                    )
                )
    with open("Matrix.pickle", "wb") as fp:  # Pickling
        pickle.dump(matrix, fp)
    return matrix


############################################################################################
# create_graph -> this function will create a graph with the matrix created by get_matrix
#                 and will add edges to the nodes that are navigable
#  matrix: matrix created by get_matrix
# to get this folowing four coordinates i used paint has
#  img_x_min: x coordinate of the top left corner of the image
#  img_y_min: y coordinate of the top left corner of the image
#  img_x_max: x coordinate of the bottom right corner of the image
#  img_y_max: y coordinate of the bottom right corner of the image
#  size_of_square: choose the size of each square you want to evaluate in pixels
#
############################################################################################


def create_graph(matrix, img_x_min, img_y_min, img_x_max, img_y_max, size_of_square):
    # put the numeber of lines that exist on the matrix
    lines = (img_x_max - img_x_min) // size_of_square
    # put the number collumns that exist in the matrix
    coordinates = (img_y_max - img_y_min) // size_of_square
    # adding nodes to graph with the networkx library
    G = nx.Graph()
    for i in range(lines):
        for j in range(coordinates):
            if matrix[i][j][2] == 1:
                G.add_node((matrix[i][j][0], matrix[i][j][1]))

    # list of operands to get the 8 surrounding coordinates
    neighbors = [
        (1, 0, 1),
        (1, -1, sqrt(2)),
        (0, -1, 1),
        (-1, -1, sqrt(2)),
        (-1, 0, 1),
        (-1, 1, sqrt(2)),
        (0, 1, 1),
    ]
    # static temporary node list in order to iterate it
    temp = list(G.nodes())
    # this function will iterate the nodes in the graph and add edges
    # to each of the surrounding coordinates you can navigate to
    for node in temp:
        for x in range(len(neighbors)):
            line = node[0] + neighbors[x][0]
            coordinate = node[1] + neighbors[x][1]
            # checks if the coordinate is inside the matrix(corners)
            if line < 0 or coordinate < 0 or line >= lines or coordinate >= coordinates:
                continue
            # checks if you can navigate to that coordinate
            elif matrix[line][coordinate][2] == 0:
                continue
            G.add_edge(
                node,
                (
                    matrix[line][coordinate][0],
                    matrix[line][coordinate][1],
                ),
                weight=neighbors[x][2],
            )
    with open("MapTest.pickle", "wb") as fp:  # Pickling
        pickle.dump(G, fp)


#########################################################################################################
# main -> example of function that will call the functions in order to create the graph and the txt file
#         with the map
#
##########################################################################################################


def main():
    img_x_min = 268
    img_y_min = 251
    img_x_max = 4682
    img_y_max = 3321
    size_of_square = 11

    matrix = get_matrix(
        img_x_min, img_y_min, img_x_max, img_y_max, "CartaNautica.png", size_of_square
    )
    create_graph(
        matrix,
        img_x_min,
        img_y_min,
        img_x_max,
        img_y_max,
        size_of_square,
    )


if __name__ == "__main__":
    main()
