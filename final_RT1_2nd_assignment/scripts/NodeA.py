"""
.. module:: turtlebot_controller
   :platform: Unix
   :synopsis: Python module for the turtlebot_controller
.. moduleauthor:: Peyman Peyvandi Pour
This node implements a controller for the turtlesim
"""
#! /usr/bin/env python3 


# import the necessary libraries
import rospy
from geometry_msgs.msg import PoseStamped 
import actionlib.msg 
import assignment_2_2022.msg 
import actionlib
import rospy
from nav_msgs.msg import Odometry
from rt1_2nd_assignment.msg import odom_custom_msg
import os

#global variable
start_description_flag=1




#target_client function
#this function is called when the user selects the target position operation
#it asks the user to enter the x and y position of the target
#it creates a goal and sends it to the action server
#the goal is of type PoseStamped
#the goal is sent to the action server using the send_goal function
#the send_goal function is a member function of the SimpleActionClient class
#the send_goal function is called with the goal as an argument
#the goal is sent to the action server using the topic /reaching_goal
#the goal is sent to the action server using the action type PlanningAction
#the action type PlanningAction is defined in the package assignment_2_2022 and file Planning.action

def target_client():
    """
    Sends a goal of type PoseStamped to the action server using the send_goal function, after getting the
    target position from the user. This function is called when the user selects the target position operation.

    Args:
        None

    Returns:
        None
    """

    x_position = input("\nPlease enter X position: ")
    y_position = input("Please enter Y position: ")

 
    x_position = int(x_position)
    y_position = int(y_position)
 
    
    print(f'\nYou entered: \nposition X: {x_position}  \nposition Y: {y_position}')
    # Creates the SimpleActionClient, passing the type of the action

 


    print("\n###############################################")
    print("\nWating for connection to the action server")

    #Wait for the server to be ready to receive goals 
    client.wait_for_server()

    #Creates a goal to send to the action server.
    goal = PoseStamped()


    goal.pose.position.x = x_position
    goal.pose.position.y = y_position


    goal = assignment_2_2022.msg.PlanningGoal(goal)

    
    #Sends the goal to the action server.
    client.send_goal(goal)
    print("\n**Goal sent to the sever**")
    input("\nPress Enter to select an operation!")

    #Back to the interface function 
    interface()
      

#cancel_target function 
#this function is called when the user selects the cancel operation
#it cancels the goal that is sent to the action server
#the goal is canceled using the cancel_goal function
#the cancel_goal function is a member function of the SimpleActionClient class

def cancel_target():
    """
    Cancels the goal that is sent to the action server using the cancel_goal function.
    This function is called when the user selects the cancel operation.

    Args:
        None

    Returns:
        None
    """
    #Cancel the goal that is sent to the action server
    client.cancel_goal()
    print(f"\nTarget canceled")
    input("\n\nPress Enter to select an operation!")

    #Back to the interface function
    interface()


#wrong function
#this function is called when the user enters a wrong input
#it prints a message and waits for 2 seconds
def wrong():
    """
    Prints a message and waits for 2 seconds. This function is called when the user enters a wrong input.

    Args:
        None

    Returns:
        None
    """

    print("!!!! Wrong input !!!!")
    rospy.sleep(2)
    interface()


#interface function
#this function is called when the program starts
#it prints the interface and asks the user to select an operation
#the user can select the target position operation, the cancel operation or the exit operation
#the user can select the target position operation by entering 1
#the user can select the cancel operation by entering 2
#the user can select the exit operation by entering 3
#the user can select the target position operation by calling the target_client function
#the user can select the cancel operation by calling the cancel_target function
#the user can select the exit operation by calling the exit function
def interface():
    """
    Prints the interface and asks the user to select an operation. The user can select the target position operation by
    entering 1, the cancel operation by entering 2, or the exit operation by entering 3. This function then calls the
    corresponding function based on the user input.

    Args:
        None

    Returns:
        None
    """

    os.system('clear')
    print("###############################################\n")    
    print("##          Robot control interface          ##\n")
    print("###############################################\n")
    print("1:Target position\n")
    print("2:Cancel\n")
    print("3:Exit\n")   

    #Ask the user to select an operation
    user_selection = input("Select your operation: ")
    
    #Check the user selection
    if   (user_selection == "1"):
        target_client()

    elif (user_selection == "2"):
        cancel_target() 

    elif (user_selection == "3"):
        exit()

    else:
        wrong()

#start_description function
#this function is called when the program starts
#it prints the description of the node
#it waits for the user to press enter
#it sets the start_description_flag to 0
#the start_description_flag is used to print the description of the node only once
def start_description(start_description_flag):
    """Prints a description of Node A and waits for user input to continue.

    Args:
        start_description_flag (int): A flag indicating whether to print the description or not.
            If the flag is 1, the description will be printed. Otherwise, nothing will be printed.

    Returns:
        None
    """
    if start_description_flag == 1:
        os.system('clear')
        print("\n\n------------------Node A description------------------\n\n")
        print("This is the node that implements an action client, ")
        print("allowing the user to set a target (x, y) or to ")
        print("cancel it.")
        print("\n\n----------------------------------------------------\n\n")
        print("Also this node publishes the robot position and velocity ")
        print("as a custom message (x,y, vel_x, vel_z), by relying ")
        print("on the values published on the topic /odom.")
        input("\n\nPress Enter to continue!")
        start_description_flag=0   


#callback function
#this function is called when the node receives a message from the topic /odom
#this fuction prints the message received from the topic /odom
#this fuction publishes the message received from the topic /odom as a custom message
#the custom message is of type odom_custom_msg
#the custom message is published on the topic /position_and_velocity
def callback(data):
    """Callback function that is called when a message is received from the /odom topic.

    The function publishes the received message as a custom message of type odom_custom_msg on the
    /position_and_velocity topic.

    Args:
        data (Odometry): The received message from the /odom topic.

    Returns:
        None
    """
    
    my_publisher = rospy.Publisher('position_and_velocity', odom_custom_msg, queue_size=5)
    my_custom_message = odom_custom_msg()
    my_custom_message.x = data.pose.pose.position.x
    my_custom_message.y = data.pose.pose.position.y
    my_custom_message.vel_x = data.twist.twist.linear.x
    my_custom_message.vel_y = data.twist.twist.linear.y

    my_publisher.publish(my_custom_message)



if __name__ == '__main__':
    #start_description_flag is used to print the description of the node only once
    start_description(start_description_flag)
    
    #Initialize the node
    #The name of the node is NodeA
    rospy.init_node('NodeA')
    #Subscribe to the topic /odom
    #The callback function is called when the node receives a message from the topic /odom
    rospy.Subscriber("/odom", Odometry, callback)

    #Creates the SimpleActionClient, passing the type of the action
    #to the constructor.
    client = actionlib.SimpleActionClient('/reaching_goal',assignment_2_2022.msg.PlanningAction )
    
    #Call the interface function
    #the interface function prints the interface and asks the user to select an operation
    #the user can select the target position operation, the cancel operation or the exit operation
    interface()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

