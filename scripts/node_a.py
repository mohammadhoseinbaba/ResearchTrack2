"""
.. module:: node_a
   :platform: Unix
   :synopsis: Python module for node_a

.. moduleauthor:: Mohammadhossein baba

This node implements an action client that allows the user to set or cancel navigation goals.
It also publishes the robot's position and velocity on the `/pos_vel` topic using a custom message.

Subscribers:
    /odom (nav_msgs/Odometry): Receives odometry data from the robot.

Publishers:
    /pos_vel (assignment_2_2023/Vel): Publishes current position and velocity.

Action Clients:
    /reaching_goal (PlanningAction): Sends goals to be reached by the robot.
"""

#!/usr/bin/env python3

try:
    import rospy
    from geometry_msgs.msg import Point, Pose, Twist
    from nav_msgs.msg import Odometry
    import actionlib
    from assignment_2_2023.msg import Vel, PlanningAction, PlanningGoal
    from actionlib_msgs.msg import GoalStatus
except ImportError:
    pass


class GoalHandler:
    """
    A class to handle user interaction for sending or canceling navigation goals,
    and publishing the robot's current position and velocity.
    """

    def __init__(self):
        """Initializes the ROS node, sets up publishers and the action client."""
        rospy.init_node('set_target_client')
        self.pub = rospy.Publisher("/pos_vel", Vel, queue_size=1)
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        self.client.wait_for_server()
        self.goal_cancelled = True

    def handle_goal_commands(self):
        """Handles user input to set or cancel a goal."""
        rospy.Subscriber("/odom", Odometry, self.publish_position_velocity)
        while not rospy.is_shutdown():
            command = input("Press 'y' to set a new goal or 'c' to cancel the current goal: ")
            target_pos_x = rospy.get_param('/des_pos_x')
            target_pos_y = rospy.get_param('/des_pos_y')

            goal = PlanningGoal()
            goal.target_pose.pose.position.x = target_pos_x
            goal.target_pose.pose.position.y = target_pos_y
            rospy.loginfo("Current goal: target_x = %f, target_y = %f", target_pos_x, target_pos_y)

            if command == 'y':
                try:
                    input_x = float(input("Enter the x-coordinate for the new goal: "))
                    input_y = float(input("Enter the y-coordinate for the new goal: "))
                except ValueError:
                    rospy.logwarn("Invalid input. Please enter a valid number.")
                    continue

                rospy.set_param('/des_pos_x', input_x)
                rospy.set_param('/des_pos_y', input_y)
                goal.target_pose.pose.position.x = input_x
                goal.target_pose.pose.position.y = input_y

                self.client.send_goal(goal)
                self.goal_cancelled = False

            elif command == 'c':
                if not self.goal_cancelled:
                    self.goal_cancelled = True
                    self.client.cancel_goal()
                    rospy.loginfo("Current goal has been cancelled")
                else:
                    rospy.loginfo("No active goal to cancel")
            else:
                rospy.logwarn("Invalid command. Please enter 'y' or 'c'.")

            rospy.loginfo("Last received goal: target_x = %f, target_y = %f",
                          goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)

    def publish_position_velocity(self, msg):
        """Callback function to publish robot's velocity and position."""
        current_pos = msg.pose.pose.position
        current_vel_linear = msg.twist.twist.linear
        current_vel_angular = msg.twist.twist.angular

        pos_and_vel = Vel()
        pos_and_vel.pos_x = current_pos.x
        pos_and_vel.pos_y = current_pos.y
        pos_and_vel.vel_x = current_vel_linear.x
        pos_and_vel.vel_z = current_vel_angular.z

        self.pub.publish(pos_and_vel)


def main():
    """Main function to start the GoalHandler logic."""
    handler = GoalHandler()
    handler.handle_goal_commands()


if __name__ == '__main__':
    main()

