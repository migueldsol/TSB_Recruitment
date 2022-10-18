# convert png to a graph according to the pixel values

import sys
import png
import numpy as np
import matplotlib.pyplot as plt


def main():
    r = png.Reader("ola3.png")  # read png file
    w, h, pixels, meta = r.read_flat()
    data = np.array(list(pixels)).reshape((h, w))

    plt.imshow(data, cmap="gray", interpolation="nearest")
    plt.show()


if __name__ == "__main__":
    main()
