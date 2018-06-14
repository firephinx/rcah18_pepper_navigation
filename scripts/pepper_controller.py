#!/usr/bin/env python

import math
import rospy
import json
import actionlib
from rcah18_pepper_msgs.msg import SpeechProcessed
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import navigation_location_parameters

class PepperController:

    def __init__(self):
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        print("Waiting for action server")

        self.client.wait_for_server()

        print("Found action server.")

        self.speech_sub = rospy.Subscriber('/pepper_speech/processed_speech', SpeechProcessed, self.speechCallback)

    def speechCallback(self, msg):
        if(msg.frame == "go_to"):
            parameters = json.loads(msg.parameters)
            if "location" in parameters:
                locations = parameters['location']
                for i in xrange(len(locations)):
                    self.determineCurrentLocation()
                    self.determineCurrentRoom()

                    self.desired_location = locations[i]
                    self.determineDesiredRoom()
                    print("Going to the " + self.desired_location + ".")

                    if(self.determineLeaveRoom() == False):
                        self.turnToLeaveRoom()

                    self.determineGoalLocation()
                    self.go_to_desired_location()
                
    def determineCurrentLocation(self):
        self.current_pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)

        self.current_location = ''
        min_dist = 3
        for key, value in navigation_location_parameters.locations.iteritems():
            dist = math.sqrt(math.pow((self.current_pose.pose.pose.position.x - value[0]), 2) + math.pow((self.current_pose.pose.pose.position.y - value[1]), 2))

            if(dist < min_dist):
                min_dist = dist
                self.current_location = key

        print("Current location: " + self.current_location)

    def determineCurrentRoom(self):
        if(self.current_location == "kitchen" or self.current_location == "kitchen_table" or self.current_location == "refrigerator" 
           or self.current_location == "kitchen_door_entrance" or self.current_location == "kitchen_door_exit"):
            self.current_room = "kitchen"
        elif(self.current_location == "living_room" or self.current_location == "futon" or self.current_location == "couch" 
             or self.current_location == "living_room_door_entrance" or self.current_location == "living_room_door_exit"):
            self.current_room = "living room"
        elif(self.current_location == "bedroom" or self.current_location == "bed" or self.current_location == "bedroom_door_entrance" 
             or self.current_location == "bedroom_door_exit"):
            self.current_room = "bedroom"
        elif(self.current_location == "front_door_entrance" or self.current_location == "front_door_exit" 
             or self.current_location == "hallway_near_front_door" 
             or self.current_location == "hallway_between_front_door_and_kitchen_and_living_room"):
            self.current_room = "hallway"
        elif(self.current_location == "outside_front_door"):
            self.current_room = "outside"

    def determineDesiredRoom(self):
        if(self.desired_location == "kitchen" or self.desired_location == "kitchen table" or self.desired_location == "refrigerator" 
           or self.desired_location == "kitchen door"):
            self.desired_room = "kitchen"
        elif(self.desired_location == "living room" or self.desired_location == "futon" or self.desired_location == "couch" 
             or self.desired_location == "living room door"):
            self.desired_room = "living room"
        elif(self.desired_location == "bedroom" or self.desired_location == "bed" or self.desired_location == "bedroom door"):
            self.desired_room = "bedroom"
        elif(self.desired_location == "front door"):
            self.desired_room = "hallway"
        elif(self.desired_location == "outside"):
            self.desired_room = "outside"

    def determineLeaveRoom(self):
        return (self.current_room == self.desired_room)

    def leaveCurrentRoom(self):
        if(self.current_room == "kitchen"):
            exit_door_location = navigation_location_parameters.kitchen_door_exit
        elif(self.current_room == "living room"):
            exit_door_location = navigation_location_parameters.living_room_door_exit
        elif(self.current_room == "bedroom"):
            exit_door_location = navigation_location_parameters.bedroom_door_exit
        elif(self.current_room == "outside"):
            exit_door_location = navigation_location_parameters.front_door_exit
        else:
            return

        self.move_base(exit_door_location)

    def turnToLeaveRoom(self):
        if(self.current_room == "kitchen"):
            exit_door_location = navigation_location_parameters.kitchen_door_exit
        elif(self.current_room == "living room"):
            exit_door_location = navigation_location_parameters.living_room_door_exit
        elif(self.current_room == "bedroom"):
            exit_door_location = navigation_location_parameters.bedroom_door_exit
        elif(self.current_room == "outside"):
            exit_door_location = navigation_location_parameters.front_door_exit
        else:
            return

        x_vector = exit_door_location[0] - self.current_pose.pose.pose.position.x
        y_vector = exit_door_location[1] - self.current_pose.pose.pose.position.y
        angle = math.atan2(y_vector, x_vector)

        rotate_to_face_the_exit_door = [self.current_pose.pose.pose.position.x, self.current_pose.pose.pose.position.y, 0.0, 
                                        0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)]

        self.move_base(rotate_to_face_the_exit_door)

    def determineGoalLocation(self):
        
        if(self.desired_location == "kitchen"):
            self.goal_location = navigation_location_parameters.kitchen
        elif(self.desired_location == "living room"):
            self.goal_location = navigation_location_parameters.living_room
        elif(self.desired_location == "bedroom"):
            self.goal_location = navigation_location_parameters.bedroom
        elif(self.desired_location == "bed"):
            self.goal_location = navigation_location_parameters.bed
        elif(self.desired_location == "futon"):
            self.goal_location = navigation_location_parameters.futon
        elif(self.desired_location == "couch"):
            self.goal_location = navigation_location_parameters.couch
        elif(self.desired_location == "refrigerator"):
            self.goal_location = navigation_location_parameters.refrigerator
        elif(self.desired_location == "kitchen table"):
            self.goal_location = navigation_location_parameters.kitchen_table
        elif(self.desired_location == "front door"):
            self.goal_location = navigation_location_parameters.front_door_entrance
        elif(self.desired_location == "kitchen door"):
            self.goal_location = navigation_location_parameters.kitchen_door_entrance
        elif(self.desired_location == "living room door"):
            self.goal_location = navigation_location_parameters.living_room_door_entrance
        elif(self.desired_location == "bedroom door"):
            self.goal_location = navigation_location_parameters.bedroom_door_entrance
        elif(self.desired_location == "outside"):
            self.goal_location = navigation_location_parameters.outside_front_door
        else:
            self.goal_location = navigation_location_parameters.front_door_entrance

    def go_to_desired_location(self):
        self.move_base(self.goal_location)

    def move_base(self, location):
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"        
        
        goal.target_pose.pose.position.x = location[0]
        goal.target_pose.pose.position.y = location[1]
        goal.target_pose.pose.position.z = location[2]
        goal.target_pose.pose.orientation.x = location[3]
        goal.target_pose.pose.orientation.y = location[4]
        goal.target_pose.pose.orientation.z = location[5]
        goal.target_pose.pose.orientation.w = location[6]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")