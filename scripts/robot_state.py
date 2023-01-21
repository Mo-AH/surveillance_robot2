#!/usr/bin/env python
"""
.. module:: robot_state
  :platform: Unix 
  :synopsis: Python module for the robot-state node
.. moduleauthor:: Mohammad Al Horany, 5271212@studenti.unige.it

This module implements the node that manages the battery level.
The battery level can be changed randomly by setting its time parameter or manually by the user.
This dual-behaviour can be switched by changing the parameter `test/random_sense/active`.

Publishes to:
    - /state/battery_low

Services:
    - /state/charge_battery

"""

import threading
import random
import rospy

# Import constant name defined to structure the architecture.
from surveillance_robot2 import architecture_name_mapper as anm

# Import the messages used by services and publishers.
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_ROBOT_STATE

class RobotState:
    """ The robot state manager class.
        This class provides:
        - service to recharge the battery
        - publisher to notify that the battery is low.
    """

    def __init__(self):
        # Initialise this node.
        rospy.init_node(anm.NODE_ROBOT_STATE, log_level=rospy.INFO)
        
        # Initialise robot position.
        self._pose = None

        # Initialise battery level and get param specifying time required for charging
        self._battery_low = False
        self.charging_time = rospy.get_param(anm.PARAM_CHARGING_TIME, 3)

        # Initialise randomness, if enabled.
        self._randomness = rospy.get_param(anm.PARAM_RANDOM_ACTIVE, True)
        if self._randomness:
            self._random_battery_time = rospy.get_param(anm.PARAM_BATTERY_TIME, [1000.0, 2000.0])
            log_msg = (f'Random-based battery low notification active: the battery change state (i.e., low/high) with a '
                       f'delay in the range of [{self._random_battery_time[0]}, {self._random_battery_time[1]}) seconds.')
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        
        # Define services.
        rospy.Service(anm.SERVER_CHARGE, SetBool, self.recharge)

        # Start publisher on a separate thread.
        th = threading.Thread(target=self.battery_publisher)
        th.start()

        # Log information.
        log_msg = (f'Initialise node `{anm.NODE_ROBOT_STATE}` with service {anm.SERVER_CHARGE} and topic {anm.TOPIC_BATTERY_LOW}.')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))


    def battery_publisher(self):
        """ Publish changes of battery levels.
            This method runs on a separate thread.
        """

        # Define a `lathed` publisher to wait for initialisation and publish immediately.
        publisher = rospy.Publisher(anm.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)
        if self._randomness:
            # Publish battery level changes randomly.
            self._random_battery_notifier(publisher)
        else:
            # Publish battery level changes through a keyboard-based interface.
            self._manual_battery_notifier(publisher)


    def _random_battery_notifier(self, publisher):
        """ Method that publish when the battery change state (i.e., high/low) based on a random delay within the interval
            [`self._random_battery_time[0]`, `self._random_battery_time[1]`).
            The message is published through the `publisher` input parameter and is a
            boolean value, i.e., `True`: battery low, `False`: battery high.

            Args:
                publisher (Publisher) : publisher to publish the boolean value of the battery.

        
        """
        delay = 0  # Initialised to 0 just for logging purposes.
        while not rospy.is_shutdown():
            
            # Publish battery level.
            publisher.publish(Bool(self._battery_low))
            
            # Log state.
            if self._battery_low:
                log_msg = f'Robot got low battery after {delay} seconds.'
            else:
                log_msg = f'Robot got a fully charged battery after {delay} seconds.'
            self._print_info(log_msg)
            
            # Wait for simulate battery usage.
            delay = random.uniform(self._random_battery_time[0], self._random_battery_time[1])
            rospy.sleep(delay)
            
            # Change battery state.
            self._battery_low = not self._battery_low


    def _manual_battery_notifier(self, publisher):
        """ Method that allows keyboard interaction to emulate battery level changes.
            The message is published through the `publisher` input parameter and is a
            boolean value, i.e., `True`: battery low, `False`: battery high.

            Args:
                publisher (Publisher) : publisher to publish the boolean value of the battery.

        
        """
        # Explain keyboard-based interaction.
        print('  # Type `Low` (`L`) to notify that the battery is low, `hiGh` (`G`) that it is fully charged.')
        print('  # Type `cnt+C` and `Enter` to quit.')

        # Publish the default value at startup.
        publisher.publish(Bool(self._battery_low))

        # Loop to enable multiple interactions.
        while not rospy.is_shutdown():
            # Wait for the user to enter a battery state.
            user_input = input(' > ')
            user_input = user_input.lower()

            # Understand the entered text.
            error = False
            if user_input == 'low' or user_input == 'l':
                self._battery_low = True
                rospy.loginfo(anm.tag_log('Robot got low battery.', LOG_TAG))
            elif user_input == 'high' or user_input == 'g':
                self._battery_low = False
                rospy.loginfo(anm.tag_log('Robot got a fully charged battery.', LOG_TAG))
            else:
                # Cannot understand the entered command.
                print('*** USER INPUT ERROR! Try again:')
                error = True

            # Publish the massage based on the entered command.
            if not error:
                publisher.publish(Bool(self._battery_low))


    def recharge(self, request):
        """ The `state/charge_battery` service implementation.
            It waits the time specified in the parameter charging_time before returning the response.

            Args:
                request (SetBoolRequest) : generic request 
            Returns:
                response (SetBoolResponse) : response that confirms the complete charging.

        """
        response = SetBoolResponse()
        if request.data == True:
            log_msg = f'[CHARGE] = Recharging...'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

            rospy.sleep(self.charging_time)
            
            log_msg = f'[CHARGE] = Robot fully charged.'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

            self._battery_low = False 
            response.success = True
        else:
            response.success = False

        return response


    def _print_info(self, msg):
        """ Print logging only when random testing is active.
            This is done to allow an intuitive usage of the keyboard-based interface.
            
            Args:
                msg (str) : message to log

        """
        if self._randomness:
            rospy.loginfo(anm.tag_log(msg, LOG_TAG))


if __name__ == "__main__":
    # Instantiate the node manager class and wait.
    RobotState()
    rospy.spin()

