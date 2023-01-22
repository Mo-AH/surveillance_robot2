#! /usr/bin/env python
"""
.. module:: arm_controller 
  :platform: Unix 
  :synopsis: Python module for the arm controller implementation
.. moduleauthor:: Mohammad Al Horany, 5271212@studenti.unige.it

This module implements a node running an action server for controlling the arm.
It provides two movements, the one to get back in home pose [0, 0, 0] and the one that
do a full rotation both looking up and looking down, to permit a full check of the room.

"""

import numpy as np
import rospy
from math import pi
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from actionlib import SimpleActionServer
from surveillance_robot2.msg import ArmControlGoal, ArmControlFeedback, ArmControlResult
import surveillance_robot2  # This is required to pass the `ArmControlAction` type for instantiating the `SimpleActionServer`.

# Arm Movements constant
BACK_TO_HOME_POSE = 0
COMPLETE_SCAN = 1
SIMPLE_ROTATION = 2

class ArmControllerAction():
	"""A class that provides an action server to control the arm movements.
		This server can rotate the arm to check all the room or can go back to the home position.
	"""
	def __init__(self):
		# Publishers to control joints position
		self.arm_base_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size = 10)
		self.arm1_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size = 10)
		self.arm2_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size = 10)

		# Initialize the trajectory's poses for rotation
		self.scan_poses = np.empty((43,3),dtype=float)
		self.initialize_scan_poses()

		# Instantiate and start the action server based on the `SimpleActionServer` class.
		self._as = SimpleActionServer("/control_arm",
										surveillance_robot2.msg.ArmControlAction,
									  execute_cb=self.execute_callback,
									  auto_start=False)
		self._as.start()


	def execute_callback(self, goal):
		"""  The callback invoked when a client set a goal to the `arm_controller` server.
			This function requires a movement code which can be 0 for the reaching of
			the home position and 1 for a round trajectory, to permit the arm to rotate
			thus the camera checks the entire room. 
			Args:
				goal(ArmControlGoal) : The action goal represented by a movement code.
		
		"""
		# Check if the provided goal is processable. If not, this service will be aborted.
		if goal is None or (goal.movement_cmd != 0 and goal.movement_cmd != 1 and goal.movement_cmd != 2):
			rospy.logerr('[@arm_controller] : No valid movement code provided!')
			self._as.set_aborted()
			return

		trajectory_poses = [[0.0, 0.0, 0.0]]

		if goal.movement_cmd == COMPLETE_SCAN:
			trajectory_poses = self.scan_poses
			print(f'[@arm_controller] : The camera arm will start rotate to perform a complete scan of the room!')
		
		elif goal.movement_cmd == SIMPLE_ROTATION:
			trajectory_poses = [[i*(2*pi/8), 0, 0] for i in range(0,9)]
			print(f'[@arm_controller] : The camera arm will start rotate to perform a quick check of the room!')

		elif goal.movement_cmd == BACK_TO_HOME_POSE:
			print(f'[@arm_controller] : The camera arm will go back to the home pose!')

		# Construct the feedback and loop for each trajectory's pose
		feedback = ArmControlFeedback()
		joint1_cmd = Float64()
		joint2_cmd = Float64()
		joint3_cmd = Float64()
		rate = rospy.Rate(2)

		# Perform the trajectory
		for pose in trajectory_poses:

			# Check that the client did not cancel this service.
			if self._as.is_preempt_requested():
				rospy.loginfo('[@arm_controller]: Service has been cancelled by the client!')
				# Actually cancel this service.
				self._as.set_preempted()
				return

			# Publish the next pose goal
			joint1_cmd = pose[0]
			joint2_cmd = pose[1]
			joint3_cmd = pose[2]
			self.arm_base_pub.publish(joint1_cmd)
			self.arm1_pub.publish(joint2_cmd)
			self.arm2_pub.publish(joint3_cmd)

			# Wait before going to the following pose
			rate.sleep()

			# Publish a feedback to the client with the current pose 
			feedback.current_pose = pose
			self._as.publish_feedback(feedback)


		# Publish the results to the client and set the action as succeded
		result = ArmControlResult()
		result.final_pose = pose
		self._as.set_succeeded(result)

		print(f'[@arm_controller] : The camera arm has terminated the movement!')
		return  


	def initialize_scan_poses(self):
		"""  Method to initialize the poses for a full rotation trajectory,
			which permits the scan of the entire room.
		
		"""
		self.scan_poses[0,:] = [0.0, 0.0, -0.2]

		# ROUND-UP
		for i in range (1,20):
			self.scan_poses[i,:] = [i*(2*pi/19), 0.0, -0.2]

		self.scan_poses[20,:] = [2*pi, 0.0, 0.0]
		self.scan_poses[21,:] = [0.0, -0.4, 0.8]

		# ROUND-DOWN
		for i in range (1,20):
			self.scan_poses[i+21,:] = [i*(2*pi/19), -0.4, 0.8]
		self.scan_poses[41,:] = [0.0, -0.4, 0.8]

		# FRONT VIEW (HOME POSE)
		self.scan_poses[42,:] = [0.0, 0, 0]



if __name__ == "__main__":

	# Initialise the node, its action server, and wait. 
	rospy.init_node("arm_controller", log_level=rospy.INFO)
	ArmControllerAction()
	rospy.spin()