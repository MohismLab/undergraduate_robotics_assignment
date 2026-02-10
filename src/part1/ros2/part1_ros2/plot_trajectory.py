from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import numpy as np
import matplotlib.pylab as plt


def main():
    path = Path(get_package_share_directory('part1_ros2')) / 'tmp' / 'webots_exercise.txt'
    data = np.genfromtxt(path, delimiter=",")

    plt.figure()
    plt.plot(data[:, 0], data[:, 1], "b", label="true")
    # For the localization exercise.
    if data.shape[1] == 6:
        plt.plot(data[:, 3], data[:, 4], "g", label="estimated")
    # Cylinder.
    a = np.linspace(0.0, 2 * np.pi, 20)
    x = np.cos(a) * 0.3 + 0.3
    y = np.sin(a) * 0.3 + 0.2
    plt.plot(x, y, "k")
    # Walls.
    plt.plot([-2, 2], [-2, -2], "k")
    plt.plot([-2, 2], [2, 2], "k")
    plt.plot([-2, -2], [-2, 2], "k")
    plt.plot([2, 2], [-2, 2], "k")
    plt.axis("equal")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.xlim([-2.5, 2.5])
    plt.ylim([-2.5, 2.5])

    if data.shape[1] == 6:
        plt.figure()
        error = np.linalg.norm(data[:, :2] - data[:, 3:5], axis=1)
        plt.plot(error, c="b", lw=2)
        plt.ylabel("Error [m]")
        plt.xlabel("Timestep")

    plt.show()


if __name__ == "__main__":
    main()
