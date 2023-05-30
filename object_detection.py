#python code for object detection

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
from queue import PriorityQueue

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
        self.bridge = CvBridge()
        self.object_cascade = cv2.CascadeClassifier('object_cascade.xml')  # Path to object detection cascade file
        self.object_detected = False

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
        img = np.uint8(data)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        img = cv2.transpose(img)
        img = cv2.flip(img, 0)
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

    def scan_callback(self, scan):
        ranges = scan.ranges
        min_range = min(ranges)
        if min_range < 0.5:
            self.object_detected = True

    def image_callback(self, image):
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        objects = self.object_cascade.detectMultiScale(gray_image, scaleFactor=1.1, minNeighbors=5)

        if len(objects) > 0:
            self.object_detected = True

    def find_object_position(self):
        rospy.loginfo("Searching for object...")
        self.object_detected = False

        while not self.object_detected:
            rospy.sleep(0.1)

        rospy.loginfo("Object detected!")

        # You can modify this logic to determine the object's position based on the detected object
        # For example, you can calculate the object's centroid from the detected bounding box

        # For demonstration, we assume the object's position is at the center of the image
        image_width = 640
        image_height = 480
        object_position = (image_width // 2, image_height // 2)

        return object_position

    def run(self):
        rospy.loginfo("Finding object position...")
        object_position = self.find_object_position()
        rospy.loginfo("Object position: {}".format(object_position))

        rospy.loginfo("Planning path...")
        start_position = (0, 0)  # Replace with the actual starting position of the robot
        path = self.astar(start_position, object_position)

        if path is not None:
            rospy.loginfo("Path found!")
            rospy.loginfo("Following the path...")
            for position in path:
                self.navigate_to_position(position)
            rospy.loginfo("Reached object position.")

            rospy.loginfo("Picking up object...")
            # Add your logic here to control the gripper arm and pick up the object

            rospy.loginfo("Dropping off object...")
            drop_off_position = (0, 0)  # Replace with the actual drop-off position
            self.navigate_to_position(drop_off_position)

            rospy.loginfo("Object dropped off.")
        else:
            rospy.logerr("Failed to find a path to the object.")

if __name__ == '__main__':
    planner = PathPlanner()
    rospy.Subscriber('/scan', LaserScan, planner.scan_callback)
    rospy.Subscriber('/camera/image_raw', Image, planner.image_callback)
    planner.run()
