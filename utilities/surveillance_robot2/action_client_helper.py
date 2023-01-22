#!/usr/bin/env python
"""
.. module:: action_client_helper 
  :platform: Unix 
  :synopsis: Python module for a simple action client helper
.. moduleauthor:: Mohammad Al Horany, 5271212@studenti.unige.it

This module implements a class that simplifies the implementation of a client for ROS action servers.
It is used by the class :mod:`smach_helper`

"""

# Import ROS libraries.
import rospy
from actionlib import SimpleActionClient

# Import mutex to manage synchronization among ROS-based threads (i.e., node loop and subscribers)
from threading import Lock

# Import constant names that define the architecture's structure.
from surveillance_robot2 import architecture_name_mapper as anm

# Import ROS-based messages.
from std_msgs.msg import Bool

# Constants
LOG_TAG = 'action-client-helper'


class ActionClientHelper:
    """ A class to simplify the implementation of a client for ROS action servers.

        Args:
            service_name (str) : it is the name of the server that will be invoked by this client.
            
            action_type (msg): it is the message type that the server will exchange.
            
            done_callback (function) : it is the name of the function called when the action server completed its computation. If
            this parameter is not set (i.e., set to `None`), then only the `self._done_callback` function will be
            called when the server completes its computation.
            
            feedback_callback (function): it is the name of the function called when the action server sends a feedback message. If
            this parameter is not set (i.e., set to `None`), then only the `self._feedback_callback` functions will be
            called when the server sends a feedback message.
            
            mutex (Lock): it is a `Lock` object synchronised with the `done_callback` and `feedback_callback`. If it is not set
            (i.e., set to `None`), then a new mutex instance is considered. Set this variable if you want to extends the
            synchronization with other classes.
    """

    def __init__(self, service_name, action_type, done_callback=None, feedback_callback=None, mutex=None):
        
        # Initialise the state of this client, i.e.,  `_is_running`, `_is_done`, and `_results`.
        self.reset_client_states()
        
        # Set the name of the server to be invoked.
        self._service_name = service_name
        
        # Get or create a new mutex.
        if mutex is None:
            self._mutex = Lock()
        else:
            self._mutex = mutex
        
        # Instantiate a simple ROS-based action client.
        self._client = SimpleActionClient(service_name, action_type)

        # Set the done and feedback callbacks defined by the class using this client.
        self._external_done_cb = done_callback
        self._external_feedback_cb = feedback_callback
        
        # Wait for the action server to be alive.
        self._client.wait_for_server()

    def send_goal(self, goal):
        """ Start the action server with a new `goal`.
            Note this call is not blocking (i.e., asynchronous performed).

            Args:
                goal (ActionGoal): the goal message of the action
        
        """
        # A new goal can be given to the action server only if it is not running. This simplification implies that
        # within the ROS architecture no more than one client can use the same server at the same time.
        if not self._is_running:
            # Start the action server.
            self._client.send_goal(goal,
                                   done_cb=self._done_callback,
                                   feedback_cb=self._feedback_callback)
            # Set the client's states.
            self._is_running = True
            self._is_done = False
            self._results = None
        else:
            warn_msg = 'Warning send a new goal, cancel the current request first!'
            rospy.logwarn(anm.tag_log(warn_msg, LOG_TAG))

    def cancel_goals(self):
        """Stop the computation of the action server.
        """

        # The computation can be stopped only if the server is actually computing.
        if self._is_running:
            # Stop the computation.
            self._client.cancel_all_goals()
            # Reset the client's state.
            self.reset_client_states()
        else:
            warn_msg = 'Warning cannot cancel a not running service!'
            rospy.logwarn(anm.tag_log(warn_msg, LOG_TAG))

    def reset_client_states(self):
        """Reset the client state variables stored in this class.
        """
        self._is_running = False
        self._is_done = False
        self._results = None


    def _feedback_callback(self, feedback):
        """This function is called when the action server send some `feedback` back to the client.
            
            Args:
                feedback (ActionFeedback): the feedback of the action
        
        """


        # Acquire the mutex to synchronise the computation concerning the `feedback` message with the other nodes of the architecture.
        self._mutex.acquire()
        try:
            # Eventually, call the method provided by the node that uses this action client to manage a feedback.
            if self._external_feedback_cb is not None:
                self._external_feedback_cb(feedback)
            # Uncomment below to log information.
            # rospy.loginfo(anm.tag_log(f'`{self._service_name}` action server provide feedback: {feedback}.', LOG_TAG))
        finally:
            # Realise the mutex to (eventually) unblock ROS-based thread waiting on the same mutex.
            self._mutex.release()


    def _done_callback(self, status, results):
        """This function is called when the action server finish its computation, i.e., it provides a `done` message.
            
            Args:
                status (ActionStatus): the state of the action
                results (ActionResult): the result of the action
        
        """
        # Acquire the mutex to synchronise the computation concerning the `done` message with the other nodes of the architecture.
        self._mutex.acquire()
        try:
            # Set the client's state
            self._is_running = False
            self._is_done = True
            self._results = results

            # Eventually, call the method provided by the node that uses this action client to manage a result.
            if self._external_done_cb is not None:
                self._external_done_cb(status, results)

        finally:
            self._mutex.release()


    def is_done(self):  # they should be mutex safe
        """ Method to check if the action is done.

            Returns:
                bool (Bool): `True` if the action server finished is computation, or `False` otherwise.
        """

        return self._is_done


    def is_running(self):
        """ Method to check if the action server is running.

            Returns:
                bool (Bool): `True` if the action server is running, or `False` otherwise.
        """
        return self._is_running

 
    def get_results(self):
        """ Method to get the results of the action.

            Returns:
                result (ActionResult): results of the action server, if any, or `None`.
        """
        if self._is_done:
            return self._results
        else:
            log_err = f'Error: cannot get result for `{self._service_name}`.'
            rospy.logerr(anm.tag_log(log_err, LOG_TAG))
            return None

