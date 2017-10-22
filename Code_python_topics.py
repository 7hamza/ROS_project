#!/usr/bin/env python

""" nav_test.py - Version 0.1 2012-01-10

    Command a robot to move autonomously among a number of goal locations defined in the map frame.
    On each round, select a new random sequence of locations, then attempt to move to each location
    in succession.  Keep track of success rate, time elapsed, and total distance traveled.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import roslib; roslib.load_manifest('projet_g1')
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt
from std_msgs.msg import String

# Fonction de callback appele quand on publie sur un des deux topic evenementiel
# Le code amene seulement le robot au noeud ou l'evenement a ete detecte
# Si l'on detecte autre chose que 1 (Il se passe quelquechose), alors on ne fait rien
def Callback(data):
    rospy.loginfo(data.data)
    if data.data == "1":

        rospy.loginfo("CALLBACK")
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                   'SUCCEEDED', 'ABORTED', 'REJECTED',
                   'PREEMPTING', 'RECALLING', 'RECALLED',
                   'LOST']

        locations = dict()
        #On donne ici la position du noeud ou l'evenement s'est produit
        locations['noeud1'] = Pose(Point(3.72488, 4.87662, 0.000), Quaternion(0.000, 0.000, -0.686, 0.728))

        sequence = ['noeud1']
        rospy.loginfo("sequence "+str(sequence))
        location = sequence[0]
        goal = MoveBaseGoal()
        goal.target_pose.pose = locations[location]
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        rospy.loginfo("Going to: " + str(location))

        move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        move_base.send_goal(goal)
        finished_within_time = move_base.wait_for_result(rospy.Duration(300))


         # Check for success or failure
        if not finished_within_time:
            move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
                rospy.loginfo("State:" + str(state))
            else:
              rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))



class NavTest():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=True)
        
        rospy.on_shutdown(self.shutdown)

        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 10)
        
        # Are we running in the fake simulator?
        self.fake_test = rospy.get_param("~fake_test", False)
        
        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        
        # Set up the goal locations. Poses are defined in the map frame.  
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.
        locations = dict()
        
        locations['porte_droite'] = Pose(Point(3.87465, 6.96316, 0.000), Quaternion(0.000, 0.000, 0.03, 1.000))
        locations['porte_gauche'] = Pose(Point(-2.12749, 5.99594, 0.000), Quaternion(0.000, 0.000, 1.000, 0.03))
        locations['couloir'] = Pose(Point(0.460575, 7.09737, 0.000), Quaternion(0.000, 0.000, 1.000, 0.03))
        locations['noeud1'] = Pose(Point(3.72488, 4.87662, 0.000), Quaternion(0.000, 0.000, -0.686, 0.728))
        locations['noeud2'] = Pose(Point(-1.62702, 4.12378, 0.000), Quaternion(0.000, 0.000, -0.686, 0.728))

        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 10 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(10))

        # subscribe au topic presence, avec un appel a la fonction de callback
        rospy.Subscriber("/presence",String, Callback)

         # subscribe au topic presence, avec un appel a la fonction de callback
        rospy.Subscriber("/button",String, Callback)
        
        rospy.loginfo("Connected to move base server")
        
        # A variable to hold the initial pose of the robot to be set by 
        # the user in RViz
        initial_pose = PoseWithCovarianceStamped()
        
        # Variables to keep track of success rate, running time,
        # and distance traveled
        n_locations = 4 
        n_goals = 0
        n_successes = 0
        i = n_locations
        distance_traveled = 0
        start_time = rospy.Time.now()
        running_time = 0
        location = ""
        last_location = ""
        
        # Get the initial pose from the user
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        self.last_location = Pose()
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        
        # Make sure we have the initial pose
        while initial_pose.header.stamp == "":
            rospy.sleep(1)
            
        rospy.loginfo("Starting navigation test")
        

        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():

            rospy.loginfo("*** MODE SEQUENTIEL")
            pub_mode = rospy.Publisher("activity_mode", String, queue_size=15)
            pub_mode.publish("SEQUENTIEL")

            # If we've gone through the current sequence,
            # start with a new random sequence
            if i == n_locations:
                i = 0
                #sequence = [sample(locations, n_locations)]

                # On remplace l'aleatoire des positions par un ordre de position bien defini
                sequence = ['porte_droite', 'porte_gauche','couloir', 'porte_gauche']
                rospy.loginfo("sequence "+str(sequence))
                # Skip over first location if it is the same as
                # the last location
                #if sequence[0] == last_location:
                #    i = 1
            
            # Get the next location in the current sequence
            location = sequence[i]
                        
            # Keep track of the distance traveled.
            # Use updated initial pose if available.
            if initial_pose.header.stamp == "":
                distance = sqrt(pow(locations[location].position.x - 
                                    locations[last_location].position.x, 2) +
                                pow(locations[location].position.y - 
                                    locations[last_location].position.y, 2))
            else:
                rospy.loginfo("Updating current pose.")
                distance = sqrt(pow(locations[location].position.x - 
                                    initial_pose.pose.pose.position.x, 2) +
                                pow(locations[location].position.y - 
                                    initial_pose.pose.pose.position.y, 2))
                initial_pose.header.stamp = ""
            
            # Store the last location for distance calculations
            last_location = location
            
            # Increment the counters
            i += 1
            n_goals += 1
        
            # Set up the next goal location
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = locations[location]
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            
            # Let the user know where the robot is going next
            rospy.loginfo("Going to: " + str(location))
            
            # Start the robot toward the next location
            self.move_base.send_goal(self.goal)
            
            # Allow 5 minutes to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300)) 
            
            # Check for success or failure
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    n_successes += 1
                    distance_traveled += distance
                    rospy.loginfo("State:" + str(state))
                else:
                  rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))
                


            # How long have we been running?
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs / 60.0
            
            # Print a summary success/failure, distance traveled and time elapsed
            rospy.loginfo("Success so far: " + str(n_successes) + "/" + 
                          str(n_goals) + " = " + 
                          str(100 * n_successes/n_goals) + "%")
            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) + 
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")
            rospy.sleep(self.rest_time)
            
    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
      
def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

if __name__ == '__main__':
    try:
        NavTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")

