"""
.. module:: node_c
   :platform: Unix
   :synopsis: ROS node for calculating average velocity and distance to the last goal

.. moduleauthor:: Mohammadhossein baba

This node subscribes to `/pos_vel` and calculates:
- The average velocity of the robot
- The distance to the last goal position

Services:
    /info_service (assignment_2_2023/Ave_pos_vel): Returns the above information.
"""

#!/usr/bin/env python3

try:
    import rospy
    import math
    from assignment_2_2023.msg import Vel
    from assignment_2_2023.srv import Ave_pos_vel, Ave_pos_velResponse
except ImportError:
    pass


class InfoService:
    """
    A class to compute and serve the robot's distance to its target and average linear velocity.
    """

    def __init__(self):
        """
        Initializes the ROS node, subscriber, and service server.
        """
        self.average_vel_x = 0
        self.distance = 0

        rospy.init_node('info_service')
        rospy.loginfo("Info service node initialized")

        rospy.Service("info_service", Ave_pos_vel, self.get_values)
        rospy.Subscriber("/pos_vel", Vel, self.get_distance_and_average_velocity)

    def get_distance_and_average_velocity(self, msg):
        """
        Callback for the /pos_vel topic. Computes the distance to goal and average velocity.

        Parameters
        ----------
        msg : assignment_2_2023.msg.Vel
            Contains the robot's position and velocity.
        """
        des_x = rospy.get_param('/des_pos_x')
        des_y = rospy.get_param('/des_pos_y')
        velocity_window_size = rospy.get_param('/window_size', 1)

        actual_x = msg.pos_x
        actual_y = msg.pos_y

        self.distance = math.dist([des_x, des_y], [actual_x, actual_y])

        if isinstance(msg.vel_x, list):
            vel_data = msg.vel_x[-velocity_window_size:]
        else:
            vel_data = [msg.vel_x]

        self.average_vel_x = sum(vel_data) / min(len(vel_data), velocity_window_size)

    def get_values(self, request):
        """
        Returns current average velocity and distance to goal.

        Parameters
        ----------
        request : Ave_pos_velRequest

        Returns
        -------
        Ave_pos_velResponse
        """
        return Ave_pos_velResponse(self.distance, self.average_vel_x)

    def spin(self):
        """Keeps the node alive."""
        rospy.spin()


if __name__ == "__main__":
    service = InfoService()

    rospy.wait_for_service('info_service')
    dist_vel_service = rospy.ServiceProxy('info_service', Ave_pos_vel)

    while not rospy.is_shutdown():
        response = dist_vel_service()
        rospy.loginfo(f"Service response:\n {response}")

    service.spin()

