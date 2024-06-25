"""
.. module:: turtlebot_controller
   :platform: Unix
   :synopsis: Python module for the turtlebot_controller
.. moduleauthor:: Peyman Peyvandi Pour
This node implements a controller for the turtlesim
"""
#!/usr/bin/env python3


# import the necessary libraries
import rospy
from std_srvs.srv import Empty,EmptyResponse
import assignment_2_2022.msg
import os



#global variables
reached_goal_counter = 0
canceled_goal_counetr = 0
sequence = 1 
start_description_flag = 1
print_flag = True

#callback function
#this function is called when the service is called
#it prints the number of goals reached and canceled
#and returns an empty response
#it also prints the sequence number of the service call
#the sequence number is a global variable that is incremented
#every time the service is called
#the sequence number is used to distinguish between different calls
#of the service
def callback_service(req):
    """
    This function is called when the service is called. It prints the number of goals reached and canceled and
    returns an EmptyResponse. The sequence number of the service call is also printed.

    Args:
        req: The request object. It has no inputs.

    Returns:
        An EmptyResponse object.
    """
    global canceled_goal_counetr , reached_goal_counter , sequence
    print("-------------------------------------")
    print(f"Sequence: {sequence}\nNumber of canceled goal: {canceled_goal_counetr}\nnumber of reached goal: {reached_goal_counter}")
    print("-------------------------------------")
    sequence += 1
    return EmptyResponse()


#callback function
#this function is called when a message is received on the topic /reaching_goal/result
#the message is of type PlanningAction/Result
#this function increments the global variables canceled_goal_counetr and reached_goal_counter
#depending on the status message
#the status is an integer that can have the following values:
#2: canceled
#3: reached
def callback_subscriber(data):
    """
    This function is called when a message is received on the topic "/reaching_goal/result". It increments the
    global variables canceled_goal_counetr and reached_goal_counter depending on the status message.

    Args:
        data: The message data. It is of type PlanningAction/Result.
    """

    if data.status.status == 2:

        global canceled_goal_counetr
        canceled_goal_counetr += 1
    
    elif data.status.status == 3:

        global reached_goal_counter
        reached_goal_counter += 1

#start_description function
#this function is called when the program starts
#it prints the description of the node
#it waits for the user to press enter
#it sets the start_description_flag to 0
#the start_description_flag is used to print the description of the node only once
def start_description(start_description_flag):
    """
    Prints the description of the Node B and waits for the user to press enter before setting the start_description_flag to 0.

    Args:
        start_description_flag (int): A flag that indicates whether the node description has already been printed.

    Returns:
        None
    """
    if start_description_flag == 1:
        os.system('clear')
        print("\n\n------------------Node B description------------------\n\n")
        print("This node is a service node that, when called,")
        print("prints the number of goals reached and canceled.")
        input("\n\nPress Enter to continue!")
        start_description_flag=0   



#main function
if __name__ == "__main__":

    #start_description_flag is used to print the description of the node only once
    start_description(start_description_flag)

    #logwarn is used to print a message in the terminal
    #the message is printed only once
    rospy.logwarn("service started")

    #Initialize the node
    #the name of the node is NodeB
    rospy.init_node('NodeB')

    #create a subscriber
    #the subscriber subscribes to the topic /reaching_goal/result
    #the message type is PlanningAction/Result
    #the callback function is callback_subscribe    
    rospy.Subscriber("/reaching_goal/result", assignment_2_2022.msg.PlanningActionResult, callback_subscriber)

    #create a service
    #the service name is reach_cancel_ints
    #the service type is Empty
    #the callback function is callback_service
    rospy.Service('reach_cancel_ints', Empty, callback_service)

    # # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

    rate = rospy.Rate(1) # 1hz
    #the node is alive until the user presses ctrl+c
    while not rospy.is_shutdown():
        
        if print_flag== True:
            print("waiting for a client to call the service ...")   
            print_flag = not print_flag
        else:
            print("waiting for a client to call the service .")
            print_flag = not print_flag
        
        rate.sleep()