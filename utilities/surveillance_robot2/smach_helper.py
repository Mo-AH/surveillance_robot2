#! /usr/bin/env python
"""

.. module:: smach_helper
  :platform: Unix 
  :synopsis: Python module for the helper class, used by the module :mod:`smach_robot`
.. moduleauthor:: Mohammad Al Horany, 5271212@studenti.unige.it

This module can be considered as the core of the state machine, by helping it to process its computation.
It implements a specific method for every state plus some other useful method.
It exchanges information with the ontology with the api provided by the `*armor_api* <https://github.com/EmaroLab/armor_py_api>`_ library.
It builds the topological map, gets the battery state and controls the robot and arm movements 
Subscriber of:
	- /state/get_battery
	- /map/connections

Service client of:
	- /state/charge_battery

Action client of:
	- /move_base
	- /control_arm

"""
import roslib
import rospy
import time
import random
import re

from armor_api.armor_client import ArmorClient
from os.path import dirname, realpath
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolRequest

import actionlib
import actionlib.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Import mutex to manage synchronization
from threading import Lock

# Internal imports
from surveillance_robot2 import architecture_name_mapper as anm
from surveillance_robot2.action_client_helper import ActionClientHelper
from surveillance_robot2.msg import RoomConnection, Room, ArmControlAction, ArmControlGoal


LOG_TAG = anm.NODE_SMACH_ROBOT
LOOP_SLEEP_TIME = 0.3


class SmachHelper():
	"""This class is the state machine helper. It loads the ontology, initializes
		useful variables and provides methods to help the state machine in controlling the robot,
		both in simulation and in the ontology. 
	
	"""
	def __init__(self):

		# Get parameters for the simulation
		self.charging_time = rospy.get_param(anm.PARAM_CHARGING_TIME, 10)

		#-----------------------#
		# 	BATTERY 			#
		#-----------------------#
		# Initialize the mutex for synchronization
		self.battery_mutex = Lock()

		# Subscribe to the battery low topic and initialize the variable
		rospy.Subscriber(anm.TOPIC_BATTERY_LOW, Bool, self.battery_callback)
		self.battery_low = False

		# Create a service client to recharge the battery
		rospy.wait_for_service(anm.SERVER_CHARGE)
		self.client_recharge = rospy.ServiceProxy(anm.SERVER_CHARGE, SetBool)

		#-----------------------#
		# 	ONTOLOGY 			#
		#-----------------------#
	  	# Initialize the mutex for synchronization
		self.map_mutex = Lock()

	  	# Start the ArmorClient and loads the ontology file
		path = dirname(realpath(__file__))
		path = path + "/../../ontologies/"
		self.armor_client = ArmorClient("surveillance_robot", "reference")
		self.armor_client.utils.load_ref_from_file(path + anm.ONTOLOGY_FILENAME, anm.ONTOLOGY_IRI, True, "PELLET", True, False)
		self.armor_client.utils.mount_on_ref()
		self.armor_client.utils.set_log_to_terminal(True)

		# Variables of the ontology map
		self.locations_list = []
		self.corridors_list = []
		self.doors_list = []
		self.current_location = anm.STARTING_LOCATION 
		self.target_location = None
		self.urgent_locations = []
		
		#-----------------------#
		# SIMULATION ENVIRONMENT#
		#-----------------------#
		self.locations_info = dict()
		self.map_built = False

		# Define the ArmController client
		self.armcontroller_client = ActionClientHelper('/control_arm', ArmControlAction)

		# Define the MoveBase client
		self.movebase_client = ActionClientHelper('move_base', MoveBaseAction)

#---------------------------------- BUILD MAP ------------------------------------#

	def build_map(self):
		""" Method paired with BUILD_MAP state.
			It assert the robot initial location, then waits for the map information making the camera-arm rotate.
			Once map has been built, it disjoints all the individuals, initializes the timer of locations and the urgency threshold. 
		
		"""

		# Assert the robot initial location
		self.armor_client.manipulation.add_objectprop_to_ind('isIn', anm.ROBOT_NAME, anm.STARTING_LOCATION)
		print(f'\n[@smach_robot - BuildMap] : ROBOT {anm.ROBOT_NAME} spawned in ROOM {anm.STARTING_LOCATION},' +
					' waiting for the information to build the map.')

		# Subscribe to the map/connections topic to get informations about the map
		rospy.Subscriber('/map/rooms', Room, self.map_callback)

		# Goal to rotate the arm or get it back in home pose
		goal = ArmControlGoal()

		# Loop for checking if the map builder has finished publishing connections 
		while not rospy.is_shutdown():
			self.map_mutex.acquire()
			try:
				# Verify if all the connections has been published
				if self.map_built is True:

					# Store the locations list
					self.locations_list = list(self.locations_info.keys())

					# Disjoint all the individuals
					self.armor_client.call('DISJOINT', 'IND', '', self.locations_list+self.doors_list+[anm.ROBOT_NAME])

					# Apply changes and calls the ontology-reasoner
					self.update_ontology()

					# Save the corridors list to avoid future queries
					self.corridors_list = self.armor_client.query.ind_b2_class('CORRIDOR')

					# Initialize the timer of the rooms and of the robot
					starting_time = str(int(time.time()))
					for location in self.locations_list:
						if location not in self.corridors_list:
							self.armor_client.manipulation.add_dataprop_to_ind("visitedAt", location, "Long", starting_time)

					self.armor_client.manipulation.add_dataprop_to_ind("now", anm.ROBOT_NAME, "Long", starting_time)
					self.last_move_time = starting_time

					# Initialize the urgency threshold of the robot
					self.armor_client.manipulation.add_dataprop_to_ind("urgencyThreshold", anm.ROBOT_NAME, "Long", anm.URGENCY_THRESHOLD)

					# Apply changes and calls the ontology-reasoner
					self.update_ontology()

					break
				else:
					# If the map is not built and the arm is still, rotate the arm
					if not self.armcontroller_client.is_running():
						goal = ArmControlGoal()
						goal.movement_cmd = 1
						self.armcontroller_client.send_goal(goal)
			finally:
				self.map_mutex.release()
			rospy.sleep(LOOP_SLEEP_TIME)


	def map_callback(self, msg):
		"""The subscriber callback of the `/map/rooms'` topic.
		It receives rooms information and store them.
				
				Args:
					msg (Room) : msg containing informations about a room
		"""
		if msg.room:

			print(f'[@smach_robot - BuildMap] : Room {msg.room} informations received.')
			self.locations_info[msg.room] = (msg.x, msg.y)

			for connection in msg.connections:

				# Adds the connection to the ontology
				self.armor_client.manipulation.add_objectprop_to_ind("hasDoor", msg.room, connection.through_door)
				self.armor_client.manipulation.add_objectprop_to_ind("hasDoor", connection.connected_to, connection.through_door)
				
				# Store the location and door list
				if connection.connected_to not in self.locations_list:
					self.locations_list.append(connection.connected_to)
				if connection.through_door not in self.doors_list:	
					self.doors_list.append(connection.through_door)

				# Log message
				print(f'[@smach_robot - BuildMap] : Connection acquired: Location {msg.room} connected to ' +
						f'{connection.connected_to} through Door {connection.through_door}.')

		# The empty message represents the end of the acquiring process
		else:
			self.map_mutex.acquire()
			self.map_built = True
			self.map_mutex.release()


#---------------------------------- REASONER ------------------------------------#

	def decide_next_location(self):
		""" Method paired with the REASONER state.

			It decides the next location by querying the ontology. The policy of the decision is the following:
				- If battery_low, next_location = charging location
				- If urgent locations nearby, next_location = most urgent reachable location
				- Else, next_location = random CORRIDOR among the reachable

		    Returns:
		    	next_location(str): location to reach

		"""
		# If the battery is low, go to room E
		self.battery_mutex.acquire()
		try:
			if self._battery_low:
				return anm.CHARGING_LOCATION
		finally:
			self.battery_mutex.release()

		# Get reachable rooms
		reachable_locations = self.armor_client.query.objectprop_b2_ind("canReach", anm.ROBOT_NAME)
		random.shuffle(reachable_locations)

		print(f'\n[@smach_robot - Reasoner] : From here, I can reach {reachable_locations}.')


		# If there are urgent rooms nearby, go to the most urgent
		next_location = self.get_most_urgent(reachable_locations)
		if next_location is not None and next_location is not anm.CHARGING_LOCATION:
			print(f'[@smach_robot - Reasoner] : Urgent room {next_location}!! Ill go there.\n')
			return next_location

    	# Otherwise go to a corridor
		else:
			for location in reachable_locations:
				if location in self.corridors_list and location is not anm.CHARGING_LOCATION:
					print(f'[@smach_robot - Reasoner] : No urgent locations here, Ill go in CORRIDOR {location}.\n')
					return location


	def get_most_urgent(self, reachable_locations):
		""" This method is used by `decide_next_location()` and it returns the most urgent location among the ones given in input, if any, otherwhise it returns False.

		    Args:
		    	reachable_locations (str[]): The list of reachable locations
		  
		    Returns:
		    	most_urgent_location(str): The most urgent location among the reachable ones; if none of them are URGENT,
		    								this is None.
		
		"""

		# Get the urgent locations
		self.urgent_locations = self.armor_client.query.ind_b2_class('URGENT')
		# Get the intersection among REACHABLE and URGENT locations
		urgent_reachable_locations = [location for location in self.urgent_locations if location in reachable_locations]

		# If there are not reachable urgent locations urgent, return None 
		if not urgent_reachable_locations:
			return None

		# Return the most urgent among the reachable urgent locations
		last_visit = 0
		most_urgent_location = urgent_reachable_locations[0]
		for location in urgent_reachable_locations:
				query = int(self.format_query(self.armor_client.query.dataprop_b2_ind("visitedAt", location), "TIMESTAMP")[0])
				if last_visit > query:
					most_urgent_location = location
					last_visit = query
		return most_urgent_location


#---------------------------------- MOVE ------------------------------------#

	def move_robot(self, new_location):
		"""Method paired with the Move state.
			It sends a goal to /move_base and checks the battery in the meanwhile.
			Once arrived, it updates the robot location in the ontology ("isIn" object prop.) and the last time it moved ("now" data prop.)

		    Args:
		    	new_location (str): The goal location

		    Returns:
		    	bool(Bool): True if it finishes correctly, False if battery got low
		
		"""
		# Set the goal for the desired room and send it
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = self.locations_info[new_location][0]
		goal.target_pose.pose.position.y = self.locations_info[new_location][1]
		goal.target_pose.pose.orientation.w = 1

		# Reset the state of previous stimuli to assure that only the new stimulus are considered.
		self.movebase_client.reset_client_states()

    	# Start the action server for moving the robot
		self.movebase_client.send_goal(goal)

    	# Wait for the action server computation and listen possible incoming stimulus.
		while not rospy.is_shutdown():

      	# Acquire the mutex to assure data consistencies
			self.battery_mutex.acquire()
			try:

				# If the battery is low, then cancel the control action goal and return False
				if self._battery_low and new_location is not anm.CHARGING_LOCATION:  # Higher priority
					self.movebase_client.cancel_goals()
					return False

        		# If the robot has finished the motion, break
				if self.movebase_client.is_done():
					break

			finally:
                
        		# Release the mutex
				self.battery_mutex.release()

      		# Wait for a reasonably small amount of time to allow the processing of battery state change
			rospy.sleep(LOOP_SLEEP_TIME)

		# Update the robot location in the ontology ("isIn" object prop.) and the last time it moved ("now" data prop.)
		move_time_new = str(int(time.time()))
		self.armor_client.manipulation.replace_objectprop_b2_ind("isIn", anm.ROBOT_NAME, new_location, self.current_location)
		self.current_location = new_location
		self.armor_client.manipulation.replace_dataprop_b2_ind("now", anm.ROBOT_NAME, "Long", move_time_new, self.last_move_time)
		self.last_move_time = move_time_new
		self.update_ontology()

		return True

#---------------------------------- CHECK LOCATION ------------------------------------#

	
	def check_location(self, location):
		"""Method paired with the CHECK_LOCATION state. It rotates the arm using the /control_arm action.
			In the meanwhile, it gets the battery state and cancels the checking if battery got low.

		    Args:
		    	location (str[]): The location to check

		    Returns:
		    	bool (Bool): True if it finishes correctly, False if battery got low
		
		"""

		# Rotate the arm action
		goal = ArmControlGoal()
		goal.movement_cmd = 2
		self.armcontroller_client.reset_client_states()
		self.armcontroller_client.send_goal(goal)

    # Wait for the action server to perform the movement and check the battery in the meanwhile
		while not rospy.is_shutdown():
      	
			self.battery_mutex.acquire()
			try:
				# If the battery is low, return
				if self._battery_low and location is not anm.CHARGING_LOCATION:  # Higher priority
					return False

        		# If the robot has finished the motion, pass to the next state
				if self.armcontroller_client.is_done():
					break
			finally:
				self.battery_mutex.release()
      		# Wait for a reasonably small amount of time to allow the processing of battery state change
			rospy.sleep(LOOP_SLEEP_TIME)
		
		# Update the "visitedAt" data property of the location when check is finished
		new_visit_time = str(int(time.time()))
		last_visit_time = self.format_query(self.armor_client.query.dataprop_b2_ind('visitedAt', location), "TIMESTAMP")[0]
		self.armor_client.manipulation.replace_dataprop_b2_ind('visitedAt', location, 'Long', new_visit_time, last_visit_time)
		self.update_ontology()

		return True

#---------------------------------- CHARGE ------------------------------------#


	def recharge(self):
		"""Method used to send a request to the service that recharges the battery.

		"""
		print(f'\n[@smach_robot - Charge] : Recharging .......')

		request = SetBoolRequest()             
		request.data = True                    
		self.client_recharge(request)  

		print(f'[@smach_robot - Charge] : Robot fully charged!')

		self.battery_mutex.acquire()
		self._battery_low = False
		self.battery_mutex.release()


#---------------------------------- BATTERY ------------------------------------#

	def battery_callback(self, msg):
		"""The subscriber of the `/state/battery_low/` topic.
		It acquires the changes of states of the battery.
				
				Args:
					msg (Bool) : represents the battery state
		"""
		# Get the battery level and set the relative state variable encoded in this class with the mutex
		self.battery_mutex.acquire()
		self._battery_low = msg.data
		self.battery_mutex.release()

		
#---------------------------------- UTILS ------------------------------------#

	def update_ontology(self):
		"""Method to apply the changes in the buffer and to synchronyze the reasoner.
			It is not protected by mutex because it is called always by the manipulations methods
			that already have the mutex.
		
		"""
		self.armor_client.utils.apply_buffered_changes()
		self.armor_client.utils.sync_buffered_reasoner()



	@staticmethod
	def format_query(old_list, type_of_object):
		"""Function to format a list of query strings, by extracting only the meaningful part.
		    
		    Args:
		       old_list (str[]): The list of queries to format.
		       type_of_object (str[]): The query type, that can be 'LOCATION' or 'TIMESTAMP'
		    
		    Returns:
		       formatted_list (str[]): The formatted list of queries.
		
		"""
		start_position = 0
		formatted_list = []
		if type_of_object == 'LOCATION':
			start_position = 32
			end_position = -1
		elif type_of_object == 'TIMESTAMP':
			start_position = 1
			end_position = -11
		for obj in old_list:
			formatted_list.append(obj[start_position:end_position])
		return formatted_list