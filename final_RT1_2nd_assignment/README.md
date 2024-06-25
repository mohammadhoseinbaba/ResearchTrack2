[Università degli studi di Genova](https://unige.it/en/ "University of Genoa")

Professor: [Carmine Recchiuto](https://github.com/CarmineD8 "Carmine Recchiuto")

Student: Mohammadhossein baba  - 5919466 - Robotics Engineering 

Final assignment of Research Track 1 course
------------
- [Aims of the assignment:](#aims-of-the-assignment-)
    * [Node A](#node-a)
    * [Node B](#node-b)
    * [Node C](#node-c)
    * [Launch file](#launch-file)
- [ROS](#ros)
    - [What is ROS?](#what-is-ros-)
    - [How ROS works?](#how-ros-works-)
        * [Nodes:](#nodes-)
        * [Topics:](#topics-)
        * [Services:](#services-)
        * [Parameter server:](#parameter-server-)
    - [ROS Tools](#ros-tools)
        * [rviz:](#rviz-)
        * [rosbag:](#rosbag-)
        * [catkin:](#catkin-)
        * [rosbash:](#rosbash-)
        * [roslaunch:](#roslaunch-)
    - [ROS Simulator](#ros-simulator)
        * [Gazebo:](#gazebo-)
- [Installing and running](#installing-and-running)
- [How it works](#how-it-works)
    * [Node A](#node-a-1)
    * [Node B](#node-b-1)
    * [Print speed](#print-speed)
- [possible improvements](#possible-improvements)



## Aims of the assignment:

Create a new package in which the following tasks should be developed: 


###### Node A 

- A node that implements an action client, allowing the user to set a target (x, y) or cancel it. The node also publishes the robot position and velocity as a custom message (x,y, vel_x, vel_z), relying on the values published on the topic /odom. 

###### Node B

- A service node that, when called, prints the number of goals reached and canceled.

###### Node C

- A node that subscribes to the robot’s position and velocity (using the custom message) and prints the distance of the robot from the target and the robot’s average speed.

###### Launch file

- A launch file that  starts the whole simulation. Also, in this launch file the value for the frequency at which node C publishes the information is set.

## ROS
##### What is ROS?
- Robot Operating System (ROS or ros) is an open-source robotics middleware suite. Although ROS is not an operating system (OS) but a set of software frameworks for robot software development, it provides services designed for a heterogeneous computer cluster such as hardware abstraction, low-level device control, implementation of commonly used functionality, message-passing between processes, and package management. Running sets of ROS-based processes are represented in a graph architecture where processing takes place in nodes that may receive, post, and multiplex sensor data, control, state, planning, actuator, and other messages. Despite the importance of reactivity and low latency in robot control, ROS is not a real-time operating system (RTOS). However, it is possible to integrate ROS with real-time computing code. The lack of support for real-time systems has been addressed in the creation of ROS 2,a major revision of the ROS API which will take advantage of modern libraries and technologies for core ROS functions and add support for real-time code and embedded system hardware.

#####  How ROS works?
- ROS processes are represented as nodes in a graph structure, connected by edges called topics. ROS nodes can pass messages to one another through topics, make service calls to other nodes, provide a service for other nodes, or set or retrieve shared data from a communal database called the parameter server. A process called the ROS Master makes all of this possible by registering nodes to itself, setting up node-to-node communication for topics, and controlling parameter server updates. Messages and service calls do not pass through the master, rather the master sets up peer-to-peer communication between all node processes after they register themselves with the master. This decentralized architecture lends itself well to robots, which often consist of a subset of networked computer hardware, and may communicate with off-board computers for heavy computing or commands.


###### Nodes:
- A node represents one process running the ROS graph. Every node has a name, which registers with the ROS master before it can take any other actions. Multiple nodes with different names can exist under different namespaces, or a node can be defined as anonymous, in which case it will randomly generate an additional identifier to add to its given name. Nodes are at the center of ROS programming, as most ROS client code is in the form of a ROS node which takes actions based on information received from other nodes, sends information to other nodes, or sends and receives requests for actions to and from other nodes.


###### Topics:
- Topics are named buses over which nodes send and receive messages. Topic names must be unique within their namespace as well. To send messages to a topic, a node must publish to said topic, while to receive messages it must subscribe. The publish/subscribe model is anonymous: no node knows which nodes are sending or receiving on a topic, only that it is sending/receiving on that topic. The types of messages passed on a topic vary widely and can be user-defined. The content of these messages can be sensor data, motor control commands, state information, actuator commands, or anything else.


###### Services:
- A node may also advertise services. A service represents an action that a node can take which will have a single result. As such, services are often used for actions that have a defined start and end, such as capturing a one-frame image, rather than processing velocity commands to a wheel motor or odometer data from a wheel encoder. Nodes advertise services and call services from one another.


###### Parameter server:
- The parameter server is a database shared between nodes which allow for communal access to static or semi-static information. Data that does not change frequently and as such will be infrequently accessed, such as the distance between two fixed points in the environment, or the weight of the robot, are good candidates for storage in the parameter server.

##### ROS Tools
- ROS's core functionality is augmented by a variety of tools which allow developers to visualize and record data, easily navigate the ROS package structures, and create scripts automating complex configuration and setup processes. The addition of these tools greatly increases the abilities of systems using ROS by simplifying and providing solutions to a number of common robotics development problems. These tools are provided in packages like any other algorithm, but rather than providing implementations of hardware drivers or algorithms for various robotic tasks, these packages provide task and robot-agnostic tools which come with the core of most modern ROS installations.

###### rviz:
- rviz is a three-dimensional visualizer used to visualize robots, the environments they work in, and sensor data. It is a highly configurable tool, with many different types of visualizations and plugins.

###### rosbag:
- rosbag is a command line tool used to record and playback ROS message data. rosbag uses a file format called bags,[71] which log ROS messages by listening to topics and recording messages as they come in. Playing messages back from a bag is largely the same as having the original nodes which produced the data in the ROS computation graph, making bags a useful tool for recording data to be used in later development. While rosbag is a command line only tool, rqt_bag provides a GUI interface to rosbag.

###### catkin:
- catkin is the ROS build system, having replaced rosbuild  as of ROS Groovy. catkin is based on CMake, and is similarly cross-platform, open-source, and language-independent.

###### rosbash:
- The rosbash package provides a suite of tools which augment the functionality of the bash shell. These tools include rosls, roscd, and roscp, which replicate the functionalities of ls, cd, and cp respectively. The ROS versions of these tools allow users to use ros package names in place of the file path where the package is located. The package also adds tab-completion to most ROS utilities, and includes rosed, which edits a given file with the chosen default text editor, as well rosrun, which runs executables in ROS packages. rosbash supports the same functionalities for zsh and tcsh, to a lesser extent.

###### roslaunch:
- roslaunch is a tool used to launch multiple ROS nodes both locally and remotely, as well as setting parameters on the ROS parameter server. roslaunch configuration files, which are written using XML can easily automate a complex startup and configuration process into a single command. roslaunch scripts can include other roslaunch scripts, launch nodes on specific machines, and even restart processes which die during execution.

##### ROS Simulator
######  Gazebo:
- Gazebo is an open-source 3D robotics simulator. It integrated the ODE physics engine, OpenGL rendering, and support code for sensor simulation and actuator control. Gazebo can use multiple high-performance physics engines, such as ODE, Bullet, etc. (the default is ODE). It provides realistic rendering of environments including high-quality lighting, shadows, and textures. It can model sensors that "see" the simulated environment, such as laser range finders, cameras (including wide-angle), Kinect style sensors, etc. For 3D rendering, Gazebo uses the OGRE engine.

## Installing and running
>It is the purpose of this assignment to develop a ROS package containing three ROS nodes that provide a way to interact with the environment presented in the [assignment_2_2022](https://github.com/CarmineD8/assignment_2_2022 "assignment_2_2022") package. Python nodes were written for the assignment, as well as the directory and the CMakeList file were modified. A launch file was also developed for executing the code.

The simulation requires the following steps for running:

- A ROS Noetic ([ROS Noetic installation instructions](http://wiki.ros.org/noetic/Installation:// "ROS Noetic installation instructions"))
- Run the ROS core by executing this command in terminal:
````shell
roscore
````
- Creat a ROS worksapace:
````shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
````
- Source the new setup.*sh file:
````shell
source ~/catkin_ws/devel/setup.bash
````
- Move to the `src` folder of the workspace:
````shell
cd ~/catkin_ws/src   
````
- Clone the package assignment_2_2022 which provides an implementation of an action server that moves a robot in the environment by implementing the bug0 algorithm:
````shell
git clone https://github.com/CarmineD8/assignment_2_2022
````
- Clone the package of my solution for this assignmebt:
````shell
git clone https://github.com/PeymanPP5530/final_RT1_2nd_assignment
````
- Then:
````shell
cd ~/catkin_ws 
catkin_make
````
- Now, it is possible to run the whole project by running the launch file:
````shell
roslaunch rt1_2nd_assignment rt1_2nd_assignment.launch
````
- As a result you have three nodes, rviz and Gazebo:
![result](https://github.com/PeymanPP5530/final_RT1_2nd_assignment/blob/main/image/Screenshot%20from%202023-01-27%2015-31-36.png?raw=true "result")
- The computational graph of the system can be found below:
![rqt graph](https://github.com/PeymanPP5530/final_RT1_2nd_assignment/blob/main/image/rosgraph.png?raw=true "rqt graph")
## How it works
### Node A
The flowchart below shows how the code of Node A is working:
![flowchart](https://github.com/PeymanPP5530/final_RT1_2nd_assignment/blob/main/image/Screenshot%20from%202023-01-26%2020-30-21.png?raw=true "flowchart")
### Node B
This node is a service node that, when called prints the number of goals reached and canceled. When this service runs it waits for a client to call it. 
![bode_b_wait](https://github.com/PeymanPP5530/final_RT1_2nd_assignment/blob/main/image/Screenshot%20from%202023-01-31%2011-26-22.png?raw=true "bode_b_wait")
For calling the service node, open a new terminal and execute this command:
````shell
rosservice call /reach_cancel_ints 
````
Then, the number of reached and canceled goal will print on the Node B terminal:
![node_b_print](https://github.com/PeymanPP5530/final_RT1_2nd_assignment/blob/main/image/Screenshot%20from%202023-01-31%2011-27-33.png?raw=true "node_b_print")
### Print speed
As part of the assignment, a ROS parameter "print_interval" is defined to indicate how fast data should be printed on Node C. The initial value for this parameter is 5 which means that Node C will print data on the terminal 5 times per second (5 Hz). There are two ways to change this value.
First one is executing this command:
````shell
cd ~/catkin_ws/src/rt1_2nd_assignment/launch
gedit rt1_2nd_assignment.launch
````
Then change the value:
````shell
<param name="print_interval" value= "5" />
````
Or you can execute this command:
````shell
rosparam set print_interval <value>
````
## possible improvements
1. In order to improve this project, it may be necessary to verify the robot's current position before sending the goal position. Since the robot is already in the goal position, no goal position needs to be sent to the action server.

1. In addition, this project can be improved by checking the last state of the robot by Node A before sending the Cancel request to the action server. That way, if the robot has already achieved the goal or the server has already received another cancel request, there is no need to send a new cancel request.

1. Furthermore, it would be useful to validate the goal position entered by the user before sending it to the action server. If this position is outside the robot's reachable position, the robot gets stuck in the loop of the `wall follow` function and the robot will infinitely follow the round wall of the canvas to avoid obstacles.
