"""

Path Planning test

CAD-IT Robotics Engineer position test case 
for mobile robot path planning algorithm

"""

import matplotlib.pyplot as plt
import math
import pickle
import time

show_animation = True
time_begin = time.perf_counter()
time_end = time_begin

class AStar:

    def __init__(self, ox, oy, grid_size, robot_radius):

        self.grid_size = grid_size * 2
        self.robot_radius = robot_radius
        self.min_x = 0
        self.min_y = 0
        self.max_x = 0
        self.max_y = 0
        self.obstacle_map = None
        self.x_width = 0
        self.y_width = 0

        # Define the movement and cost of the node search in the right, up, left, down, down left, up left, down right, up right directions
        # Array format: [dx, dy, cost]
        # cost = sqrt(dx^2 + dy^2)
        self.motion = [[1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1], [-1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)], [1, -1, math.sqrt(2)], [1, 1, math.sqrt(2)]]

        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, prev_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.prev_index = prev_index

    def path_planning(self, sx, sy, gx, gy):
        
        # Initializa nodes of the start and goal position
        start_node = self.Node(self.calc_xy_index(sx, self.min_x), self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x), self.calc_xy_index(gy, self.min_y), 0.0, -1)

        # Define open set (Unvisited Nodes) and closed set (Visited nodes)
        open_set = dict()
        closed_set = dict()
        open_set[self.calc_grid_index(start_node)] = start_node
        node_count = 0

        goal_found = False

        while not goal_found:

            current_id = min(open_set, key = lambda k: open_set[k].cost + self.calc_heuristic(goal_node, open_set[k])) # 
            current = open_set[current_id]
            node_count = node_count + 1

            # Plot currently evaluated nodes
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x), self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                goal_node.prev_index = current.prev_index
                goal_node.cost = current.cost
                print("Total nodes visited to reach goal: ", node_count,"nodes")

                goal_found = True

            # Remove visited node from the open set
            del open_set[current_id]

            # Add visited node to the closed set
            closed_set[current_id] = current

            # Generate new nodes on the the right, up, left, down, down left, up left, down right, up right directions of current node
            for i, n in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0], current.y + self.motion[i][1], current.cost + self.motion[i][2], current_id)
                n_id = self.calc_grid_index(node)

                # Verify current node is not an obstacle and not out of bounds
                if not self.verify_node(node):
                    continue

                # Check if node is already visited before
                if n_id in closed_set:
                    continue

                if n_id not in open_set: 
                    open_set[n_id] = node
                else:
                    # Replace the cost to move to node in open set if it is higher than the newly generated node
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node

        # Generate the final shortest route from start position to goal
        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # Generate the final shortest route from start position to goal
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        prev_index = goal_node.prev_index
        total_distance = closed_set[prev_index].cost

        while prev_index != -1:
            n = closed_set[prev_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            prev_index = n.prev_index

        
        time_end = time.perf_counter()
        time_interval = time_end - time_begin
        print("Shortest path distance from start to goal point: ", round(total_distance,3), "m")
        print("Total time: ", round(time_interval,3), "s")

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        # Calculate heuristic value based on the Euclidean distance between the current node and the goal
        h = math.sqrt(pow(n1.x - n2.x, 2) + pow(n1.y - n2.y, 2))
        return h

    def calc_grid_position(self, index, min_position):
        # Calculate node position relative to the bounds of the obstacle maze position
        pos = index * self.grid_size + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        # Generate index for start and goal node
        return round((position - min_pos) / self.grid_size)

    def calc_grid_index(self, node):
        # Generate index for nodes
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        # Verify current node is not an obstacle and not out of bounds
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        # Calculate the bounds of the obstacle positions
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))

        self.x_width = round((self.max_x - self.min_x) / self.grid_size)
        self.y_width = round((self.max_y - self.min_y) / self.grid_size)

        # Generate obstacle map relative to the bounds of the obstacle positions 
        self.obstacle_map = [[False for n in range(self.y_width)] for n in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 160.0  # [m]
    gy = 60.0  # [m]
    grid_size = 1.0  # [m]
    robot_radius = 1.0  # [m]

    # read python dict back from the file
    pkl_file = open('./obstacle.pkl', 'rb') # Open the file containing the coordinates of obstacles
    obs_coor = pickle.load(pkl_file) 
    pkl_file.close()    
    ox, oy = obs_coor['ox'], obs_coor['oy'] # Define arrays containing x-y coordinates of obstacles
    
    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "or", markersize=13)
        plt.plot(gx, gy, "om", markersize=13)
        plt.grid(True)
        plt.axis("equal")
        
    a_star = AStar(ox, oy, grid_size, robot_radius)
    rx, ry = a_star.path_planning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")    
        plt.pause(0.01)
        plt.show()
    

if __name__ == '__main__':
    main()
