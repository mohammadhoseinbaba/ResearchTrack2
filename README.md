📄 **Live Documentation:**  
👉 [View the Sphinx-generated documentation here](https://mohammadhoseinbaba.github.io/ResearchTrack2/)

#  Research Track 1 - Second Assignment
# Mohammadhossein baba 5919466

The second assignment for the Research Track involves developing a package that interacts with a simulated robot in Gazebo. This package serves as a ROS (Robot Operating System) node to control the robot's navigation system. The package consists of three nodes:

Node A: Implements an action client allowing users to set or cancel a target (x, y) and utilizes feedback/status from the action server to determine when the target is reached. Additionally, it publishes the robot's position and velocity using a custom message (x, y, vel_x, vel_z) based on values from the /odom topic.

Node B: A service node that, when called, returns the coordinates of the last target set by the user.

Node C: Another service node that subscribes to the robot's position and velocity using a custom message. It implements a server to retrieve the distance of the robot from the target and the robot's average speed.

The package also includes a launch file for starting the entire simulation, with a parameter to select the size of the averaging window for Node C.


Figure 1: The flowchart of the first node

Documentation
To clone the code and the entire package from the GitHub repository into a ROS workspace, use the following command in your terminal (clone it into a ROS workspace). You can also find documentation about the code in the provided link.

```bash
$ git clone https://github.com/nimaabaeian/RT-Assignment2
```
Package Structure
The main folders and files in the assignment_2_2023 package are as follows:

src Directory:

- go_to_point_service.py: Node for making the robot move to a specified location.
- wall_follow_service.py: Node for making the robot move forward to reach a wall and then follow the wall until it passes it.
- bug_as.py: Node implementing an algorithm to reach a goal by combining the go_to_point and wall_follow algorithms and switching between the two cases.
- my_node_A.py: Manages the robot's position and velocity, allows dynamic changes and cancellations of goals, and publishes the robot's position and velocity using a custom message type.
- my_node_B.py: Node offering a service to retrieve the last target coordinates entered by the user.
- my_node_C.py: Offers a service to calculate and provide the average distance and velocity of the robot based on its position and velocity information and the window defined in the launch file.
- CMakeLists.txt: Configuration file for building the package containing all other packages and dependencies.
- package.xml: Package manifest file.
  
  msg Directory:
- Vel.msg: Custom ROS message structure definition representing the robot's position and velocity.

 srv Directory:
- LastTarget.srv: ROS service definition for retrieving the last entered target coordinates.
- Average.srv: ROS service definition for calculating and providing the distance to the target and average velocity.

launch Directory:
- assignment1.launch: ROS launch file executing various nodes and parameters needed for simulation.
Custom Message
- Vel.msg
- Represents the robot's position and velocity structure for the custom message.

Fields:
- x: x-coordinate of the robot's position.
- y: y-coordinate of the robot's position.
- vel_x: linear velocity of the robot in the x-axis.
- vel_z: angular velocity of the robot in the z-axis.
Custom Services
- LastTarget.srv

ROS service structure for the last entered target coordinates by the user.
Fields:
- target_x: x-coordinate of the last target entered.
- target_y: y-coordinate of the last target entered.
- Average.srv

ROS service structure that calculates and provides the average distance and velocity of the robot based on the average window.

Fields:
- dist: Distance between the robot and the target to be reached.
- velocity_mean: Mean velocity of the robot.
Launch File
assignment1.launch
Executes various nodes for simulation.
Includes simulation parameters like target x and y, and also the size of the averaging window.
Configures the simulation environment.
Launches nodes for wall following, point navigation, bug algorithm, and custom nodes for position, target retrieval, and average calculations.
Usage
Ensure that ROS is installed on your system.
Build the package using catkin_make.
Customize the parameters in assignment1.launch based on your requirements.
Before starting the simulation, make sure to make all the Python codes in the script executable by running the following code in your terminal:
```bash
$ chmod +x name_of_the_file.py
```
Run the launch file using roslaunch your_package_name assignment1.launch.
Coding
my_node_A
Functions
publish_kinematics

Subscribes to the /odom topic to obtain the current position and velocity of the robot.
Publishes the robot's position and velocity on the /kinematics topic using a custom message type named Vel.
Goal Management:

Initializes an action client for reaching goals.
Prompts the user to change the goal or cancel the movement.
Retrieves the current target from parameters in the launch file.
Allows the user to set new goals dynamically and sends the updated goal for action execution.
Cancels the current goal if requested.
Parameters
/des_pos_x: Parameter for the desired x-coordinate of the goal position.
/des_pos_y: Parameter for the desired y-coordinate of the goal position.
Usage
During runtime, the node will prompt the user to change the goal or cancel the movement. Enter "Change" to set a new goal dynamically or "Cancel" to stop the movement. The goal should be cancelled first every time we need to change it. The chosen or updated goal, as well as the robot's position and velocity, will be logged.

my_node_B
Functions
Service Implementation:
Implements the last_target service to provide the last entered target coordinates.
Retrieves the values of /des_pos_x and /des_pos_y parameters and sets them as the response to the service call.
Prints a message indicating the start of the service.
Parameters
/des_pos_x: Parameter for the desired x-coordinate of the goal position.
/des_pos_y: Parameter for the desired y-coordinate of the goal position.
Usage
The node will log a message indicating that it is ready to provide the last target coordinates. The service can be called from other nodes or services to retrieve the last entered target coordinates.

my_node_C
Functionality
Service Implementation:
Implements the average service to calculate and provide the average distance and velocity of the robot.
Retrieves the last entered target coordinates from the last_target service.
Subscribes to the /kinematics topic to obtain the current position and velocity of the robot.
Calculates the distance to the last target and maintains a list of velocities for averaging.

Average Calculation:
Calculates the Euclidean distance to the last target.
Maintains a list of velocities and calculates the average velocity over a defined window.
Parameters
/average_window: Parameter defining the size of the window for calculating the average velocity.
Usage
The node will log a message indicating that it is ready to calculate the average. The service can be called from other nodes or services to obtain the calculated average distance and velocity. It should be noted that the service can be simply called in the terminal by the following line to get the required outputs. I have commented the line related to printing the output so in case you need to know the distance to target and the average velocity simply write the following line in the terminal:


```bash
rosservice call /average

```
