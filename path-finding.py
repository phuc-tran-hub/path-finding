#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Phuc Tran
# Date: October 12th, 2021
# PA3

# Importing general libraries
import math
import numpy
import tf
from collections import deque

# Importing rospy and publisher/subscriber messages libraries 
import rospy 
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from visualization_msgs.msg import Marker

DEFAULT_GRID = 'map'
DEFAULT_POSE = 'pose_sequence'
DEFAULT_MARKER = 'marker'

FREQUENCY = 10

class Grid:
	def __init__(self, grid_data, width, height, resolution):

		# resharping the grid_data into a 2D matrix
		self.grid = numpy.reshape(grid_data, (height, width))

		# unit: cells
		self.width = width	
		self.height = height

		# Understanding the size of each grid (m/cell)
		self.resolution = resolution

	def get_cell(self, x, y):
		"""Getting a position tuple."""
		return self.grid[y, x]

class CreatePoses: 
	def __init__(self):
		"""Constructor."""

		# Map holds occupancy grid
		self.map = None
		self.map_frame = None

		# Setting up publishers and subscribers
		self.listener = tf.TransformListener()
		self._pose_pub = rospy.Publisher(DEFAULT_POSE, PoseStamped, queue_size=1)
		self._marker_pub = rospy.Publisher(DEFAULT_MARKER, Marker, queue_size=1)
		self._grid_sub = rospy.Subscriber(DEFAULT_GRID, OccupancyGrid, self._map_callback, queue_size=1)


		self.epsilon = 20
	
	def _map_callback(self, msg):
		"""Creating an OccupancyGrid object."""

		# Navigate to http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
		# to find more information about the message
		self.map_frame = msg.header.frame_id
		self.map = Grid(msg.data, msg.info.width, msg.info.height, msg.info.resolution)
	
	def bfs(self, start, goal):
		"""Exploring the map with a breadth-first search algorithm."""

		currNode = None

		# a frontier frontier to track unexplored nodes and a backpointer dictionary to track possible backpointers
		frontier = deque()
		backpointer = {}

		# Initializing the first node
		frontier.append(start)

		while currNode != goal and len(frontier) > 0:
			currNode = frontier.popleft()

			# By the 2D array logic, priority based on HW4
			# Right: to the right of the x index (x + 1)
			# Down: to the bottom of the y index (y + 1)
			# Up: to the top of the y index (y - 1)
			# Left: to the left of the x index (x - 1)
			x = currNode[0]
			y = currNode[1]

			right = self.map.get_cell(x + 1, y)
			down = self.map.get_cell(x, y + 1) 
			up = self.map.get_cell(x, y - 1)
			left = self.map.get_cell(x - 1, y)

			# Checking requirements
			# 1. if the coordinate is passed a border
			# 2. if the coordinate's occupancy probability is less than the desired probability
			# 3. the coordinate is not an obstacle
			# 4. if the the backpointer is not explored

			# Checking in order of right, bottom, top, and left backpointer
			if x <= self.map.width and right <= self.epsilon and right != -1 and backpointer.get((x + 1, y)) == None:
					newbackpointer = (currNode[0] + 1, currNode[1])
					backpointer[newbackpointer] = currNode
					frontier.append(newbackpointer)

			if y + 1 <= self.map.height and down <= self.epsilon and down != -1 and backpointer.get((x, y + 1)) == None:
					newbackpointer = (currNode[0], currNode[1] + 1)
					backpointer[newbackpointer] = currNode
					frontier.append(newbackpointer)

			if y - 1 >= 0 and up <= self.epsilon and up != -1 and backpointer.get((x, y - 1)) == None:
					newbackpointer = (currNode[0], currNode[1] - 1)
					backpointer[newbackpointer] = currNode
					frontier.append(newbackpointer)

			if x - 1 >= 0 and left <= self.epsilon and left != -1 and backpointer.get((x - 1, y)) == None:
					newbackpointer = (currNode[0] - 1, currNode[1])
					backpointer[newbackpointer] = currNode
					frontier.append(newbackpointer)

		path = []

		# Creating the path from the goal to the start
		if currNode == goal:
			while currNode != start:
				path.append(currNode)
				if currNode in backpointer:
						currNode = backpointer[currNode]

		# Reversing the path for pose_sequence message
		reversed_path = path[::-1]
		return reversed_path

	def calculate_poses(self, path):
		"""Creates a sequence of poses from a path to be published to pose_sequence"""
		# A sequence of poses
		poses = []
		for i in range(1, len(path)):
			node1 = path[i-1]
			node2 = path[i]
			point1 = [node1[0], node1[1], 0]
			point2 = [node2[0], node2[1], 0] 

			# Finding the distance the robot needs to traveled as well as the angle relative to the origin of the map
			distance = [point2[0] - point1[0], point2[1] - point1[1], 0]
			distance_unit = distance / numpy.linalg.norm(distance)
			angle = math.acos(numpy.dot(numpy.array([1,0,0]), distance_unit))

			# Finding the rotational transformation of the robot according the angle
			quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)

			# Calculating the point of the robot based on the map's resolution (a general formula)
			currentPoint = Point(point1[0] * self.map.resolution, point1[1] * self.map.resolution, 0)
			pose = Pose()
			pose.position = currentPoint

			# Pulling the orientation of the robot based on the rotational transformation
			pose.orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
			print(point1[0], point1[1], angle)
			poses.append(pose)  

		return poses

	def publish_pose(self, pose):
		"""Publishing a PoseStamped() message."""

		# Creating the PoseStamped message to publish
		posestamp_msg = PoseStamped()
		posestamp_msg.header.frame_id = self.map_frame
		posestamp_msg.pose = pose
		
		self._pose_pub.publish(posestamp_msg)


	def publish_poses(self, poses):
		"""Publishing a sequence of PoseStamped() messages."""

		rate = rospy.Rate(FREQUENCY)
		poseIndex = 0
		while not rospy.is_shutdown() and poseIndex < len(poses):
			self.publish_pose(poses[poseIndex])
			poseIndex += 1
			rate.sleep()

	def publish_marker(self, i, pose):
		"""Publishing a Marker() messages."""
		# Refer to http://wiki.ros.org/rviz/DisplayTypes/Marker#Message_Parameters
		# for Marker's message parameters
		marker = Marker()
		marker.ns = DEFAULT_GRID
		marker.id = i
		marker.type = Marker.ARROW
		
		marker.pose = pose
		marker.scale.x = 0.5
		marker.scale.y = 0.5
		marker.scale.z = 0.1

		marker.color.r = 1
		marker.color.r = 0.5
		marker.color.r = 0.5

		marker.color.a = 1
		marker.action = 0

		marker.lifetime = rospy.Duration(0)
		marker.header.stamp = rospy.Time.now()
		marker.header.frame_id = self.map_frame

		self._marker_pub.publish(marker)

	def publish_markers(self, poses):
		"""Publishing a Marker() messages."""

		rate = rospy.Rate(FREQUENCY)

		# Setting an ID for each marker
		id = 0
		while not rospy.is_shutdown() and id < len(poses):
			self.publish_pose(poses[id])
			self.publish_marker(id, poses[id])
			id += 1
			rate.sleep()

	def path_plan(self, start, goal):
			"""Create a path and visualize the path"""
			path = self.bfs(start, goal)
			poses = self.calculate_poses(path)
			self.publish_markers(poses)

def main():
	rospy.init_node("backpointer_planning")

	rospy.sleep(2)

	poses = CreatePoses()
	rospy.sleep(5)

	try:
		start = (35,15)
		goal = (79,12)
		poses.path_plan(start, goal)
	except rospy.ROSInterruptException:
		rospy.logerr("ROS node interrupted.")

if __name__ == "__main__":
    """Run the main function."""
    main()