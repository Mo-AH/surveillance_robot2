#!/usr/bin/env python
"""
.. module:: architecture_name_mapper
  :platform: Unix 
  :synopsis: Python module for the architecture name mapper
.. moduleauthor:: Mohammad Al Horany, 5271212@studenti.unige.it

In this module are defined the names of all parameters, nodes, topics, services and action servers of the architecture.
Also, it stores the data information about the ontology and the map.

"""

import rospy

#---------------------------------------------------------
# PARAMETERS
# ---------------------------------------------------------
# The name of a boolean parameter to active random testing.
# If the value is `False` a keyboard-based interface will be used to produce stimulus 
# (i.e., speech, gesture and battery signals). Instead, random stimulus will be generate 
# if `True`. In the latter case, the architecture also requires all the parameters 
# with a the scope `test/random_sense/*`, which are not used if `False`.
PARAM_RANDOM_ACTIVE = 'test/random_sense/active'

# Parameter to set the time required for charging the battery (in seconds)
PARAM_CHARGING_TIME = 'test/charging_time'

# Topic in which are sent the room informations
TOPIC_CONNECTIONS = '/map/rooms'

#---------------------------------------------------------
# ONTOLOGY
# ---------------------------------------------------------
ONTOLOGY_FILENAME = 'topological_map.owl'
ONTOLOGY_IRI = 'http://bnc/exp-rob-lab/2022-23'
ROBOT_NAME = 'AlphaBot'
STARTING_LOCATION = 'E'
CHARGING_LOCATION = 'E'
URGENCY_THRESHOLD = '180'

#---------------------------------------------------------
# NODES NAMES
# ---------------------------------------------------------
# Node representing the shared knowledge required for this scenario.
NODE_ROBOT_STATE = 'robot_state'
# Node implementing the Finite State Machine of the robot
NODE_SMACH_ROBOT = 'smach_robot'


# ---------------------------------------------------------
#  BATTERY
# ---------------------------------------------------------
# The name of the topic where the battery state is published.
TOPIC_BATTERY_LOW = 'state/battery_level'
# The name of the service to recharge the battery
SERVER_CHARGE = 'state/charge_battery'
# The delay between changes of battery levels, i.e., high/low.
# It should be a list `[min_time, max_time]`, and the battery level change
# will occur after a random number of seconds within such an interval.
PARAM_BATTERY_TIME = 'test/battery_time'



# -------------------------------------------------

def tag_log(msg, producer_tag):
    """Function used to label each log with a producer tag.
    
        Args:
            msg (str): message to print
            producer_tag (str): name of the node producer
        Returns:
            string (str): msg tagged by producer
    """
    return '@%s>> %s' % (producer_tag, msg)
