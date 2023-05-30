# python code for path planning

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import numpy as np
from PIL import Image
from queue import PriorityQueue
import cv2

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # Cost from start to current node
        self.h = 0  # Heuristic cost (estimated cost from current node to goal)
        self.f = 0  # Total cost (g + h)

    def __lt__(self, other):
        return self.f < other.f

class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner')
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.map_resolution = 0.05  # Resolution of the occupancy grid map
        self.map_origin_x = 0.0  # X-coordinate of the map origin
        self.map_origin_y = 0.0  # Y-coordinate of the map origin
        self.map = None

    def get_map(self):
        rospy.wait_for_service('static_map')
        try:
            get_map = rospy.ServiceProxy('static_map', GetMap)
            response = get_map()
            self.map = response.map
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def occupancy_grid_to_image(self):
        width = self.map.info.width
        height = self.map.info.height
        data = np.array(self.map.data).reshape((height, width))
        img = Image.fromarray(data.astype(np.uint8), mode='L')
        img = img.transpose(Image.FLIP_TOP_BOTTOM)
        return img

    def heuristic_cost(self, position, goal):
        return np.linalg.norm(np.array(position) - np.array(goal))

    def get_neighbors(self, position):
        neighbors = []
        rows, cols = self.map.shape
        r, c = position

        if r > 0 and self.map[r - 1, c] == 0:  # Up
            neighbors.append((r - 1, c))
        if r < rows - 1 and self.map[r + 1, c] == 0:  # Down
            neighbors.append((r + 1, c))
        if c > 0 and self.map[r, c - 1] == 0:  # Left
            neighbors.append((r, c - 1))
        if c < cols - 1 and self.map[r, c + 1] == 0:  # Right
            neighbors.append((r, c + 1))

        return neighbors

    def reconstruct_path(self, current_node):
        path = []
        while current_node is not None:
            path.append(current_node.position)
            current_node = current_node.parent
        return path[::-1]

    def astar(self, start, goal):
        rows, cols = self.map.shape
        open_list = PriorityQueue()
        start_node = Node(start)
        goal_node = Node(goal)
        open_list.put((start_node.f, start_node))

        visited = set()
        visited.add(start)

        while not open_list.empty():
            current_node = open_list.get()[1]
            current_position = current_node.position

            if current_position == goal:
                return self.reconstruct_path(current_node)

            neighbors = self.get_neighbors(current_position)
            for neighbor_position in neighbors:
                if neighbor_position not in visited:
                    neighbor_node = Node(neighbor_position, current_node)
                    neighbor_node.g = current_node.g + 1
                    neighbor_node.h = self.heuristic_cost(neighbor_position, goal)
                    neighbor_node.f = neighbor_node.g + neighbor_node.h

                    open_list.put((neighbor_node.f, neighbor_node))
                    visited.add(neighbor_position)

        return None

    def navigate_to_position(self, position):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.orientation.w = 1.0
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

    def find_object_position(self):
        self.get_map()
        map_image = self.occupancy_grid_to_image()

        # Convert the map image to OpenCV format
        map_image_cv = np.array(map_image)

        # Apply object detection algorithm to find the object in the map image
        # Replace the code below with your specific object detection algorithm
        object_cascade = cv2.CascadeClassifier('object_cascade.xml')  # Path to object detection cascade file
        gray_image = cv2.cvtColor(map_image_cv, cv2.COLOR_BGR2GRAY)
        objects = object_cascade.detectMultiScale(gray_image, scaleFactor=1.1, minNeighbors=5)

        if len(objects) > 0:
            # Object detected, return the position of the first object found
            x, y, w, h = objects[0]
            object_position = (x + w/2, y + h/2)
        else:
            # Object not found, return a default position
            object_position = (0, 0)

        return object_position

    def run(self):
        rospy.loginfo("Finding object position...")
        object_position = self.find_object_position()
        rospy.loginfo("Object position: {}".format(object_position))

        rospy.loginfo("Moving to object position...")
        self.navigate_to_position(object_position)
        rospy.loginfo("Reached object position.")

if __name__ == '__main__':
    planner = PathPlanner()
    planner.run()
