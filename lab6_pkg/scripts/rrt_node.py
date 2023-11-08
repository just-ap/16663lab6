#!/usr/bin/env python3
"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
from numpy import linalg as LA
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
import random
import copy

# TODO: import as you need

# class def for tree nodes

# It's up to you if you want to use this
class RRTNode(object):
    def __init__(self):
        self.x = None
        self.y = None
        self.parent = None
        self.cost = None # only used in RRT*
        self.is_root = False
        

# class def for RRT
class RRT(Node):
    def __init__(self):
        # topics, not saved as attributes
        super().__init__('rrt_node')
        # TODO: grab topics from param file, you'll need to change the yaml file
        pose_topic = "ego_racecar/odom"
        scan_topic = "/scan"
        goal_topic = "/goal_node"

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.
        self.lastTime = 0
        self.lastTimePose = 0
        self.goalSampleRate = 10
        self.gridSize = 100
        self.resolution = 0.05
        self.maxIter = 100
        self.expandDis = 0.5
        self.waypoint = []
        self.lookaheadPP = 1.5
        # TODO: create subscribers
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.pose_sub_ = self.create_subscription(
            #PoseStamped,
            Odometry,
            pose_topic,
            self.pose_callback,
            1)
        self.pose_sub_

        self.scan_sub_ = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            1)
        self.scan_sub_

        self.goal_node_pub = self.create_publisher(
            Marker,
            goal_topic,
            1)
        
        self.path_pub = self.create_publisher(
            MarkerArray,
            "/selected_path",
            1
        )

        self.ackermann_pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)


        # publishers
        # TODO: create a drive message publisher, and other publishers that you might need
        self.Ogrid_pub = self.create_publisher(OccupancyGrid, 'Ogrid', 10)

        filename = '/home/tianhao/sim_ws/src/f1tenth_lab6/lab6_pkg/waypoint/1waypoints.csv'
        with open (filename, 'r') as f:
            lines = f.readlines()
            self.wp = []
            

            # print(wp[0][0])
            for line in lines:
                count = 0
                tempx = ''
                tempy = ''
                for i in range (0, len(line) -1):
                    if count == 0 and line[i] != ',':
                        tempx += line[i]
                    elif count == 1 and line[i] != ',':
                        tempy += line[i]
                    else:
                        count += 1

                self.wp.append([tempx, tempy])

            print(self.wp)

        self.lookahead = 2.5
        # class attributes
        # TODO: maybe create your occupancy grid here
        self.occupancyGrid = None

    def scan_callback(self, scan_msg):
        interval = scan_msg.header.stamp.nanosec - self.lastTime

        if interval > 1e7 or interval < 0:
            self.lastTime = scan_msg.header.stamp.nanosec
            self.occupancyGrid = np.zeros((self.gridSize, self.gridSize)) #res = 1cm
            # for m in range(0, int(self.gridSize/2)):
            #     tempx = int(m * math.sqrt(3))
            #     for n in range(0, self.gridSize):
            #         if n > self.gridSize/2 - tempx and n < self.gridSize/2 + tempx:
            #             self.occupancyGrid[m + int(self.gridSize/2)][n] = 1

            for i in range (0, len(scan_msg.ranges)):
                x = self.gridSize/2 - scan_msg.ranges[i] * math.sin(math.pi/4 + (i + 1) * scan_msg.angle_increment) * 20
                y = scan_msg.ranges[i] * math.cos(math.pi/4 + (i + 1) * scan_msg.angle_increment)* 20+ self.gridSize

                for j in range (0, 6):
                    for k in range (0, 6):
                        self.occupancyGrid[min(max(0, int(y - j)),self.gridSize - 1)][min(max(0, int(x - k)),self.gridSize - 1)] = 1
                        self.occupancyGrid[min(max(0, int(y + j)),self.gridSize - 1)][min(max(0, int(x + k)),self.gridSize - 1)] = 1

            msg = OccupancyGrid()
            msg.header.frame_id = 'ego_racecar/base_link'
            msg.info.width = self.gridSize
            msg.info.height = self.gridSize
            msg.info.resolution = self.resolution
            msg.info.origin.position.x =  0.0
            msg.info.origin.position.y = -2.5
            # msg.info.origin.orientation.w = 1.0

            msg.data = [0] * (msg.info.width * msg.info.height)
            # print(int(self.occupancyGrid[250][250]))
            for i2 in range (0, self.gridSize):
                for j2 in range (0, self.gridSize):
                    # print(int(self.occupancyGrid[i2][j2]))
                    msg.data[self.gridSize * self.gridSize - 1 -((self.gridSize - 1 - j2) * self.gridSize + i2)] = int(self.occupancyGrid[i2][j2]*100)

            


            self.Ogrid_pub.publish(msg)
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """

    def pose_callback(self, pose_msg):
        carx = pose_msg.pose.pose.position.x
        cary = pose_msg.pose.pose.position.y
        if self.waypoint != []:
            self.pure_pursuit(carx, cary)
        interval = pose_msg.header.stamp.nanosec - self.lastTimePose
        if interval > 1e7 or interval < 0:
            self.waypoint = []
            self.lastTimePose = pose_msg.header.stamp.nanosec
            if self.occupancyGrid is None:
                return None

            tree = []
            self.startPT = RRTNode()
            self.startPT.x = 0.0
            self.startPT.y = 0.0
            self.startPT.is_root = True
            self.goalPT = self.setGoal(carx, cary)

            tree.append(self.startPT)
            path = None


            for i in range(0, self.maxIter):
                xt, yt = self.sample()
                sampling = RRTNode()
                sampling.x = xt
                sampling.y = yt
                nrIdx = self.nearest(tree, sampling)
                nearestNode = tree[nrIdx]
                newNode = self.steer(nearestNode, sampling)
                collision = self.check_collision(nearestNode, newNode)
                if collision:
                    tree.append(newNode)
                    nearGoal = self.is_goal(newNode, self.goalPT.x, self.goalPT.y)
                    if nearGoal:
                        if self.check_collision(newNode, self.goalPT):
                            lastIdx = len(tree) - 1
                            path = self.find_path(tree, tree[lastIdx])
                            self.visualize_path(path)
                            break

            if path != None:
                for i in range(0, len(path)):
                    point = PointStamped()
                    point.header.frame_id = "ego_racecar/base_link"
                    point.point.x = float(path[i][0])
                    point.point.y = float(path[i][1])
                    point.point.z = 0.0
                    try:
                        t = self.tf_buffer.lookup_transform("map", "ego_racecar/base_link", rclpy.time.Time())
                        pose_transformed = tf2_geometry_msgs.do_transform_point(point, t)
                        # self.get_logger().info(f"Transformed pose path: {pose_transformed}")
                        self.waypoint.append([pose_transformed.point.x, pose_transformed.point.y])
                        # print(pose_transformed.point.y)
                    except TransformException as ex:
                        self.get_logger().info(
                        f'Could not transform')
    


        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """

        return None
    
    def pure_pursuit(self, posx, posy):
        dis = []
        print(self.waypoint)
        for i in range (len(self.waypoint)):
            tempdis = math.sqrt((posx - float(self.waypoint[i][0]))**2 + (posy - float(self.waypoint[i][1]))**2)
            # print(tempdis)
            dis.append(tempdis)
        closeIdx = dis.index(min(dis))

        disl = []
        for i in range (len(dis)):
            temp = abs(dis[i] - self.lookaheadPP)
            disl.append(temp)
        if closeIdx > 1:
            closeIdxLook = disl.index(min(disl[0: closeIdx]))
        else:
            closeIdxLook = 0
        # print(closeIdxLook)
        point = PointStamped()
        point.header.frame_id = "map"
        point.point.x = float(self.waypoint[closeIdxLook][0])
        point.point.y = float(self.waypoint[closeIdxLook][1])
        point.point.z = 0.0

        try:
            t = self.tf_buffer.lookup_transform("ego_racecar/base_link", "map", rclpy.time.Time())
            pose_transformed = tf2_geometry_msgs.do_transform_point(point, t)
            self.get_logger().info(f"Transformed pose: {pose_transformed}")
            print(pose_transformed.point.y)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform')
            return
        
        angle = 2 * pose_transformed.point.y/(self.lookaheadPP ** 2)
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = 0.7
        self.ackermann_pub.publish(drive_msg)
    
    def visualize_path(self, path):
        # Create a MarkerArray
        marker_array = MarkerArray()

        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i+1]

            # LINE STRIPS
            line_marker = Marker()
            line_marker.header.frame_id = "ego_racecar/base_link"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.id = i
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            point1 = Point()
            point1.x=path[i][0]
            point1.y=path[i][1]
            point2 = Point()
            point2.x = path[i+1][0]
            point2.y = path[i+1][1]
            line_marker.points.append(point1)
            line_marker.points.append(point2)
            line_marker.scale.x = 0.08  # Line width
            line_marker.color.r = 0.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0
            line_marker.lifetime.sec = 1
            marker_array.markers.append(line_marker)
        self.path_pub.publish(marker_array)
    
    def setGoal(self, carx, cary):
        goalNode = RRTNode()
        tempWP = []

        for i in range (len(self.wp)):
            point = PointStamped()
            point.header.frame_id = "map"
            point.point.x = float(self.wp[i][0])
            point.point.y = float(self.wp[i][1])
            point.point.z = 0.0

        
            try:
                t = self.tf_buffer.lookup_transform("ego_racecar/base_link", "map", rclpy.time.Time())
                pose_transformed = tf2_geometry_msgs.do_transform_point(point, t)
                # self.get_logger().info(f"Transformed pose: {pose_transformed}")
                # transfer waypoint to car frame, find the point in freespace to be the goal
                if pose_transformed.point.y > -2.5 and pose_transformed.point.y < 2.5 and pose_transformed.point.x > 0.0 and pose_transformed.point.x < 5.0:
                    tempWP.append([pose_transformed.point.x, pose_transformed.point.y])
                    # print(tempWP)
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform')
        
        # print(tempWP)
        dis = []
        for i in range (len(tempWP)):
            tempdis = abs(math.sqrt(float(tempWP[i][0])**2 + float(tempWP[i][1])**2) - self.lookahead)

            dis.append(tempdis)

        # print(dis)
        if len(tempWP) == 0:
            goalNode.x = 1.0
            goalNode.y = 0.0
            return goalNode
        elif len(tempWP) > 1:
            closeIdx = dis.index(min(dis))
            print(closeIdx)
        else:
            closeIdx = 0

        marker = Marker()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        # marker.id = self.m_marker_idx
        #self.m_marker_idx += 1
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "ego_racecar/base_link"
        marker.pose.position.x = tempWP[closeIdx][0]
        marker.pose.position.y = tempWP[closeIdx][1]
        marker.pose.position.z = 0.25
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.goal_node_pub.publish(marker)

        goalNode.x = tempWP[closeIdx][0]
        goalNode.y = tempWP[closeIdx][1]

        # closeIdx = dis.index(min(dis))
        
        # print(closeIdx)
        return goalNode


    def sample(self):
        idx = np.where(self.occupancyGrid == 0)
        # print("1111111")
        # print(len(idx[0]))
        if random.randint(0, 100) > self.goalSampleRate:
            rnd = random.randint(0, len(idx[0]) - 1)
            x = idx[0][rnd] * self.resolution
            y = idx[1][rnd] * self.resolution - self.gridSize * self.resolution/2
            # print(x, "||", y)

        else:
            x = self.goalPT.x
            y = self.goalPT.y
            # print("goal" , x, "||", y)

        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        return (x, y)

    def nearest(self, tree, sampled_point):
        dis = [(node.x - sampled_point.x)**2 + (node.y - sampled_point.y) for node in tree]
        nearest_node = dis.index(min(dis))
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        # print(nearest_node)
        return nearest_node

    def steer(self, nearest_node, sampled_point):
        theta = math.atan2(sampled_point.y - nearest_node.y, sampled_point.x - nearest_node.x)
        new_node = copy.deepcopy(nearest_node)
        new_node.x += self.expandDis * math.cos(theta)
        new_node.y += self.expandDis * math.sin(theta)

        new_node.parent = nearest_node
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        # new_node = None
        return new_node

    def check_collision(self, nearest_node, new_node):
        if nearest_node == None or new_node == None:
            return False
        # new_node.y = -0.52
        # new_node.x = 1.94
        # nearest_node.y = -0.70
        # nearest_node.x = 1.47
        if new_node.x == nearest_node.x:
            return False
        slope = (new_node.y - nearest_node.y)/(new_node.x - nearest_node.x)
        # print("slope", slope)
        # print("y", int(new_node.y * 100), "||", int(nearest_node.y * 100))
        # print("x", int(new_node.x * 100), "||", int(nearest_node.x * 100))
        # if new_node.x < nearest_node.x:
        # print(nearest_node.x, "||||", new_node.x)
        for i in range(min(int(nearest_node.x * 20), int(new_node.x * 20)), max(int(nearest_node.x * 20), int(new_node.x * 20))):
            # print("i=", i)
            tempy = int((i - nearest_node.x* 20) * slope + nearest_node.y * 20 + self.gridSize/2)
            tempx = int(self.gridSize - 1 - i)
            # print("trans", tempx, "||", tempy)
            if self.occupancyGrid[min(max(0,tempx), 99)][min(max(0,tempy), 99)] == 1:
                # print("fffffff")
                return False
        
        # print("ttttt")
        return True
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        return True

    def is_goal(self, latest_added_node, goal_x, goal_y):
        d = math.sqrt((goal_y - latest_added_node.y)**2 + (goal_x - latest_added_node.x)**2)
        if d < self.expandDis:
            return True
        return False
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """
        return False

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = [[self.goalPT.x, self.goalPT.y]]
        while latest_added_node.parent is not None:
            node = latest_added_node
            path.append([node.x, node.y])
            latest_added_node = node.parent
        path.append([self.startPT.x, self.startPT.y])
        # print(path)
        return path



    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        return 0

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return 0

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        neighborhood = []
        return neighborhood

def main(args=None):
    rclpy.init(args=args)
    print("RRT Initialized")
    rrt_node = RRT()
    rclpy.spin(rrt_node)

    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
