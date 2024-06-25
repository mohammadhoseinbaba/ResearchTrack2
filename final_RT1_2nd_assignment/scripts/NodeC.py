"""
.. module:: turtlebot_controller
   :platform: Unix
   :synopsis: Python module for the turtlebot_controller
.. moduleauthor:: Peyman Peyvandi Pour

NodeC subscribes to the robot's position and velocity using a custom message, and prints the distance of the robot
from the target and the average speed of the robot. The node calculates the average velocity over the last 5 messages,
and the distance of the robot from the target is calculated using the Euclidean distance formula. The desired position
is obtained from the parameter server, which is set in the launch file. The node also prints the sequence number and
the data print interval, which is set using the "print_interval" parameter in the launch file.
"""


#!/usr/bin/env python3

# import the necessary libraries
import math
from rt1_2nd_assignment.msg import odom_custom_msg
import rospy
import os

#global variables
start_description_flag=1
counter =0
temp_vel =0
avg_vel =0
des_pos_distance=0
sequence = 1




#callback function
#this function is called when a message is received on the topic /odom_custom
#the message is of type odom_custom_msg
#this function calculates the average velocity of the robot
#the average velocity is calculated over the last 5 messages
#the average velocity is printed by the node
#the distance of the robot from the target is calculated and printed by the node
#the distance is calculated using the euclidean distance formula
def callback_subscriber(data):
    """
    This function is called when a message is received on the
    topic "/odom_custom." This function calculates the distance of the robot from the target and the current velocity
    of the robot using the received message.
    Args:
        data: 

    Returns:
        An EmptyResponse object.
    """

    #global variables
    #the global keyword is used to modify the global variables
    #inside the function
    global counter
    global temp_vel
    global avg_vel
    global des_pos_distance

    #get the desired position from the parameter server
    #the desired position is set in the launch file
    #the desired position is used to calculate the distance of the robot from the target
    #the x coordinate of the desired position is stored in the des_pos_x variable
    #the y coordinate of the desired position is stored in the des_pos_y variable
    des_pos_x = rospy.get_param("/des_pos_x")
    des_pos_y = rospy.get_param("/des_pos_y")

    #get the current position of the robot
    cur_pos_x = data.x
    cur_pos_y = data.y

    #calculate the distance of the robot from the target
    des_pos_distance= math.sqrt(((des_pos_x - cur_pos_x)**2)+((des_pos_y - cur_pos_y)**2))


    #current velocity is stored in the cur_vel variable
    cur_vel_x = data.vel_x
    cur_vel_y = data.vel_y

    #calculate the current velocity of the robot
    cur_vel= math.sqrt(((cur_vel_x)**2)+((cur_vel_y)**2))


    #calculate the average velocity of the robot
    #the average velocity is calculated over the last 5 messages
    #the average velocity is stored in the avg_vel variable
    if counter<5:

        temp_vel=temp_vel+cur_vel
        counter +=1

    elif counter==5:

        counter=0
        temp_vel /= 5
        avg_vel=temp_vel
        temp_vel=0


#start_description function
#this function is called when the program starts
#it prints the description of the node
#it waits for the user to press enter
#it sets the start_description_flag to 0
#the start_description_flag is used to print the description of the node only once
def start_description(start_description_flag):
    if start_description_flag == 1:
        os.system('clear')
        print("\n\n------------------Node C description------------------\n\n")
        print("This node subscribes to the robot’s position and ")
        print("velocity (using the custom message) and prints the ")
        print("distance of the robot from the target and the ")
        print("robot’s average speed. ")
        print("You can set the \"print_interval\" parameter in ")
        print("rt1_2nd_assignment launch flie to set how fast the")
        print("node publishes the information.")
        

        input("\n\nPress Enter to continue!")
        start_description_flag=0   


    


#main function
if __name__ == "__main__":

    #start_description_flag is used to print the description of the node only once
    start_description(start_description_flag)

    #logwarn is used to print a message in the terminal
    #the message is printed only once
    rospy.logwarn("NodeC started")

    #Initialize the node
    #the name of the node is NodeC
    rospy.init_node('NodeC')

    #create a rate object
    #the rate is set using the parameter /print_interval
    #the parameter is set in the launch file
    #the parameter is used to set the rate at which the node publishes the information
    HZ=rospy.get_param("/print_interval")
    rate = rospy.Rate(HZ)

    #create a subscriber object
    #the subscriber subscribes to the topic /odom_custom
    #the message type is odom_custom_msg
    #the callback function is callback_subscriber
    rospy.Subscriber("position_and_velocity", odom_custom_msg, callback_subscriber)

    #the node runs until it is shutdown
    #the node prints the sequence number
    #the node prints the data print interval
    #the node prints the distance of the robot from the target and the average velocity of the robot
    #the node sleeps for the time set in the rate object
    while not rospy.is_shutdown():
        print(f"Sequence: {sequence}")
        print(f"Data print interval : {HZ} HZ")        
        print(f"distance to target: {des_pos_distance : .3f}")
        print(f'Robot’s average velocity: {avg_vel: .3f}')
        print(f"---------------------------")
        sequence += 1
        rate.sleep()
