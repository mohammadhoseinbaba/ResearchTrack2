"""
.. module:: node_b
   :platform: Unix
   :synopsis: Python module for the last_target service node

.. moduleauthor:: Mohammadhossein baba

This node implements a ROS service that provides the last goal (x, y) set by the user via ROS parameters.

Services:
    /input (assignment_2_2023/Input): Returns the most recent target coordinates from the parameter server.
"""

#!/usr/bin/env python3

try:
    import rospy
    from assignment_2_2023.srv import Input, InputResponse
except ImportError:
    pass


class LastTargetService:
    """
    A class that handles a ROS service to return the last user-defined target position.
    """

    def __init__(self):
        """
        Initializes the ROS node and advertises the /input service.
        """
        self.last_des_x = 0
        self.last_des_y = 0

        rospy.init_node('last_target_service')
        rospy.loginfo("Last target node initialized")

        rospy.Service('input', Input, self.result_callback)

    def result_callback(self, request):
        """
        Service callback to retrieve and return the last target position.

        Parameters
        ----------
        request : assignment_2_2023.srv.Input
            The service request (not used).

        Returns
        -------
        assignment_2_2023.srv.InputResponse
            Contains the x and y values of the last target from the ROS parameter server.
        """
        response = InputResponse()
        self.last_des_x = rospy.get_param('/des_pos_x')
        self.last_des_y = rospy.get_param('/des_pos_y')
        response.input_x = self.last_des_x
        response.input_y = self.last_des_y

        return response

    def spin(self):
        """
        Keeps the node alive and responsive to service calls.
        """
        rospy.spin()


if __name__ == "__main__":
    service = LastTargetService()
    service.spin()

