from queue import Queue

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


class PathPlanner:
    """
    This is the class that implements the brushfire algorithm to find the path given a query.
    """

    def __init__(self, map_file: str, map_scale: float):
        # 0 represents free space, 1 represents obstacles
        self.map = np.array(Image.open(map_file))[..., 0] == 0
        self.map_scale = map_scale  # actual size(m) per pixel

    def get_distance_grid(self):
        rows, cols = self.map.shape
        distance_grid = np.full((rows, cols), np.inf)
        q = Queue()

        # Initialize the queue with obstacle locations and set their distances to 0
        for r in range(rows):
            for c in range(cols):
                if self.map[r, c] == 1:  # Obstacle cell
                    distance_grid[r, c] = 0
                    q.put((r, c))

        # Define possible movements: up, down, left, right
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        # Perform BFS to propagate distances
        while not q.empty():
            current_r, current_c = q.get()
            current_distance = distance_grid[current_r, current_c]

            for dr, dc in directions:
                new_r, new_c = current_r + dr, current_c + dc

                if 0 <= new_r < rows and 0 <= new_c < cols and distance_grid[new_r, new_c] == np.inf:
                    distance_grid[new_r, new_c] = current_distance + 1
                    q.put((new_r, new_c))

        return distance_grid

    def bfs_path(self, grid, start, goal):
        rows, cols = grid.shape
        visited = np.full((rows, cols), False)
        q = Queue()
        q.put((start, [start]))  # Store the current position and the path to reach it
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                      (-1, -1), (1, 1), (1, -1), (-1, 1),]

        while not q.empty():
            (current_r, current_c), path = q.get()

            if current_r == goal[0] and current_c == goal[1]:
                return path

            for dr, dc in directions:
                new_r, new_c = current_r + dr, current_c + dc
                if 0 <= new_r < rows and 0 <= new_c < cols and grid[new_r, new_c] == 1 and not visited[new_r, new_c]:
                    visited[new_r, new_c] = True
                    q.put(((new_r, new_c), path + [(new_r, new_c)]))

        return None  # No path found

    def plot_grid(self, grid, title="Grid"):
        plt.figure(figsize=(10, 5))
        plt.imshow(grid * self.map_scale, cmap="viridis", origin="upper")
        plt.title(title)
        cbar = plt.colorbar()
        cbar.ax.set_ylabel('Distance (m)', rotation=270)
        cbar.ax.yaxis.set_label_coords(4,0.5)
        plt.savefig('distance grid.png')
        plt.show()

    def plot_path(self, path, title="Path on Grid"):
        path_grid = 1 - np.copy(self.map)
        path_grid = np.stack([path_grid, path_grid, path_grid], axis = -1)
        print(path_grid.shape)
        for r, c in path:
            # if self.map[r, c] != 1:  # Don't overwrite obstacles
            path_grid[r-1:r+2, c-1:c+2, :] = np.array([1, 0, 0])  # Mark the path with a different value
        plt.figure(figsize=(10, 5))
        plt.imshow(path_grid*255., origin="upper")
        plt.title(title)
        plt.savefig(f"{title}.png")
        plt.show()


if __name__ == "__main__":
    brushfire = PathPlanner(map_file="./map_files/map_file_test.png", map_scale=0.3 / 100)

    # Compute the Brushfire distance grid
    distance_grid = brushfire.get_distance_grid()

    # Plot the result
    # brushfire.plot_grid(distance_grid, title="Brushfire Distance Grid")

    # Find the path
    start = np.array([350, 711])  # (y, x)
    goal = np.array([796, 1207])
    car_size = 0.2
    path = brushfire.bfs_path(distance_grid>130, start, goal)
    # path = brushfire.bfs_path(distance_grid>(car_size/brushfire.map_scale), start, goal)
    print(np.array(path) * brushfire.map_scale)
    # Plot the path
    brushfire.plot_path(path, title="Maximum safety path")
    # brushfire.plot_path(path, title="Minimum distance path")
