#! /usr/bin/env python
"""
.. module:: smach_robot 
  :platform: Unix 
  :synopsis: Python module for the state machine implementation
.. moduleauthor:: Mohammad Al Horany, 5271212@studenti.unige.it

This module implements a node running the state machine of the robot.
It manages the transitions between states, leaving the computation processes to the helper
class of the smach given by the module :mod:`state_machine_helper`.

"""
import roslib
import rospy
import smach
import smach_ros
import time
import random
from surveillance_robot2 import architecture_name_mapper as anm
from surveillance_robot2.smach_helper import SmachHelper


class BuildMap(smach.State):
    """This class defines the state BUILD_MAP of the state machine.
    """
    def __init__(self, smach_helper):
        smach.State.__init__(self, 
                             outcomes=['map_built']) 
        self.smach_helper = smach_helper
        
    def execute(self, userdata):
        """Execute method of the state BUILD_MAP
        
            Returns:
                string (str): transition to the next state.
        """

        # Build the map
        self.smach_helper.build_map()

        # Log message
        print(f'[@smach_robot - BuildMap] = Map built!! \n Locations = {self.smach_helper.locations_list} \n Corridors = ' +
                    f'{self.smach_helper.corridors_list} \n Charging location = {anm.CHARGING_LOCATION}\n')
        rospy.sleep(3)

        return 'map_built'
    

class Reasoner(smach.State):
    """This class defines the REASONER state
    """

    def __init__(self, smach_helper):
        smach.State.__init__(self, 
                             outcomes=['goal_decided'],
                             input_keys=['goal_location'],
                             output_keys=['goal_location'])
        self.smach_helper = smach_helper

    def execute(self, userdata):
        """Execute method of the REASONER state
        
            Returns:
                string (str): transition to the next state.
        """
        # Decide next location and pass it to Move state
        userdata.goal_location = self.smach_helper.decide_next_location()

        # If it's leading to the charging location, log the message
        if userdata.goal_location == anm.CHARGING_LOCATION:
            print(f'\n[@smach_robot - Reasoner] = Battery low!! Ill go in ROOM {anm.CHARGING_LOCATION}.\n')
            
        return 'goal_decided'


class Move(smach.State):
    """This class defines the state MOVE of the state machine.
    """
    def __init__(self, smach_helper):
        smach.State.__init__(self, 
                             outcomes=['location_not_urgent_reached',
                                        'location_urgent_reached',
                                        'charging_station_reached',
                                        'battery_low'],
                             input_keys=['goal_location'],
                             output_keys=['current_location'])
        self.smach_helper = smach_helper

    def execute(self, userdata):
        """Execute method of the state Move.
        
            Returns:
                string (str): transition to the next state.
        """
        # Log Message
        print(f'\n[@smach_robot - Move] = Moving to {userdata.goal_location}... ')

        # If the battery got low while moving, transit to REASONER state
        if not self.smach_helper.move_robot(userdata.goal_location):
            print(f'[@smach_robot - Move] = Battery got low while moving !!\n')
            return 'battery_low'

        userdata.current_location = userdata.goal_location

        # If it the location reached is the charging one, transit to the CHARGE state
        if userdata.goal_location == anm.CHARGING_LOCATION:
            print(f'[@smach_robot - Move] = Charging station {userdata.goal_location} reached!\n')
            return 'charging_station_reached'

        # If the location reached is not urgent, don't check it
        if userdata.goal_location not in self.smach_helper.urgent_locations:
            print(f'[@smach_robot - Move] = Location {userdata.goal_location} reached. It is not urgent, so I will skip the checking!\n')
            return 'location_not_urgent_reached'

        # Log Message
        print(f'[@smach_robot - Move] = Location {userdata.goal_location} reached!\n')

        return 'location_urgent_reached'


class CheckLocation(smach.State):
    """This class defines the state CHECK_LOCATION of the state machine.
    """
    def __init__(self,smach_helper):
        smach.State.__init__(self, 
                             outcomes=['battery_low', 'check_complete'],
                             input_keys=['current_location'])
        self.smach_helper = smach_helper


    def execute(self, userdata):
        """Execute method of the state CHECK_LOCATION.

            Returns:
                string (str): transition to the next state.
        """
        # Log Message
        print(f'\n[@smach_robot - CheckLocation] = Starting the check of location {userdata.current_location}...')

        # If the battery got low while checking, transit to REASONER state
        if not self.smach_helper.check_location(userdata.current_location):
            print(f'[@smach_robot - CheckLocation] = Battery got low while checking !!\n')
            return 'battery_low'
        
        # Log message
        print(f'[@smach_robot - CheckLocation] = Location {userdata.current_location} checked\n! ')

        return 'check_complete'



class Charge(smach.State):
    """This class defines the state CHARGE of the state machine.
    """
    def __init__(self,smach_helper):
        smach.State.__init__(self, outcomes=['charge_complete'])
        self.smach_helper = smach_helper

    def execute(self, userdata):
        """Execute method of the state CHARGE.
        
            Returns:
                string (str): transition to the next state.
        """

        # Call the service to recharge the battery
        self.smach_helper.recharge()

        return 'charge_complete'

def main():
    rospy.init_node('robot_states') #inizializza il nodo

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])


    # Create the smach_helper for the state machine
    smach_helper = SmachHelper()

    # Open the container (the first state is the one that starts)
    with sm:
        
        # Add states to the container
        smach.StateMachine.add('BUILD_MAP',BuildMap(smach_helper),
                               transitions={'map_built':'REASONER'})

        smach.StateMachine.add('REASONER', Reasoner(smach_helper),                                
                                   transitions={'goal_decided':'MOVE'})

        smach.StateMachine.add('MOVE', Move(smach_helper), 
                                transitions={'charging_station_reached': 'CHARGE',
                                            'location_not_urgent_reached' : 'REASONER',
                                            'location_urgent_reached':'CHECK_LOCATION',
                                            'battery_low':'REASONER'})
        
        smach.StateMachine.add('CHECK_LOCATION', CheckLocation(smach_helper),                    
                               transitions={'battery_low': 'REASONER',
                                            'check_complete': 'REASONER'})
        
        smach.StateMachine.add('CHARGE', Charge(smach_helper),                                 
                               transitions={'charge_complete': 'REASONER'})        


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
