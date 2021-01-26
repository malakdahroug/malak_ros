


# Malak Dahroug - PDE3432 Mobile Robots and Manipulators - Autonomous robot in ROS
Set of ROS packages forming an autonomous robot for the purpose of PDE3432 Mobile Robots and Manipulators assessment. All the packages are described in detail below.

Robot is simulated in Gazebo and programmed using ROS and ROS Python (2.7).

Robot uses AMCL, differential drive controller, move_base, find_object_2d ROS packages to fulfill robot autonomy. 

Additionally it uses various ROS plugins (also included underneath). 

All of these will have to be installed if not present in the system.

Robot is programmed using ROS Python to follow a given path, recognize objects and grasp any of the recognised balls and take them to the goal. 

Main package that will start Gazebo, spawn the balls, spawn the robot, start navigation stack and start visual recognition. In order to that repository has to be cloned to a catkin workspace source folder using following commands:

    cd your_catkin_folder/src
    git clone https://github.com/malakdahroug/malak_ros.git .

After repository is sucessfully cloned you have to run catkin make and source the new setup file. This can be done using the following commands.

    cd your_catkin_folder
    catkin_make
    source devel/setup.bash
After successfully making packages, you have to run the following command to start the full system.

    roslaunch malak_gui malak.launch

# PACKAGES:

## Assesment_world

Package provided that contains the basic world with obstacles and a goal for balls. Balls are spawned and then their positions are changed through pseudo random generated numbers.

**assessment_world.launch** – it launches the empty world and spawns turtlebot3_world inside it

**objects.launch** – spawns 3 balls (small, medium, large) and barriers of the goal. After spawning it is running move_semi_random script to move balls in a pseudo random manner.

## Malak_amcl

Package that implements AMCL (Adaptive Monte Carlo Localization) algorithm. It uses known map, laser scan and transforms to work out the position of the robot on the known map. This package is not used anymore as it was just used for testing AMCL on its own. It got implemented in Malak_nav package

#### AMCL Package contains two launch files:

**amcl.launch** – it launches standalone AMCL package, it requires map server, correct transforms and laser scan to work. If it is run on it’s own without map server it is not going to work. It takes two parameters, use_map_topic (set to true by default) and scan_topic (set to /malakrobo/laser/scan topic by default)

**amcl_demo.launch** – a complete launch file that has everything that AMCL needs to work. It takes a single argument map_file (set to path of a robots map by default), it runs map_server with the map from map_file argument, it starts AMCL navigation (include amcl.launch) and it creates a map to odom transform – this is necessary for correct robot localization.

In AMCL package there is map folder containing two maps, the map that is currently used is map2.yaml

## Malak_autonomous

This package contains functions responsible for the current and initial autonomous operation of the robot. It contains 3 launch files.

**2d_object.launch** – used by the main robot control package(Malak_gui), it is responsible for visual recognition. It runs find_object_2d package. Find_object_2d is a ROS package for visual recognition. This file is currently being used. It takes two arguments:

 - ***teach*** – responsible for opening GUI and not recognising object. If teach is set to true, it opens find_object_2d application used for training the visual recognition. You have to move robot (in Gazebo) around until objects are in the scene. Once you see the object and want to teach it to the robot you click on the navbar Edit and click “Add object from scene”.
 - ***recognize*** – responsible for starting visual recognition. If set to true recognized objects will be published on /objects topic.

In normal robot operation teach is set to false and recognize is set to true.

**operated_points.launch** – This runs python script pose_next.py, it reads the file(poses_new in scripts folder) containing Poses and executes them. It is waiting for the user to press enter before sending the next Goal. It is not used anymore – it was only for testing and learning purposes.

**points.launch** – This runs python script point_list.pt, it reads the file(poses_new in scripts folder) containing poses and executes them. It will automatically proceed to the next Pose as soon as the robot reaches the current Pose goal.

#### Package has following scripts in scripts folder:

**action_controller.py** – this is translated script from C++. [https://husarion.com/tutorials/ros-tutorials/4-visual-object-recognition/#getting-position-of-recognized-object](https://husarion.com/tutorials/ros-tutorials/4-visual-object-recognition/#getting-position-of-recognized-object)

It is working out how object is offset towards the center of the camera. It also controls the robot to position itself so it is facing the detected ball. If the ball is in the center of camera it is slowly moving towards it. This script is not used anymore, but logics were reused in the main control file.

**echo_to_list.py** – not used anymore. I created this file in a trial to convert a file input to a list of positions and orientations. This was later reused in point_list.py and pose_next.py

**point_list.py** – not used anymore. I created this to test AMCL and navigation of the robot. As well as to workout points that robot has to navigate to in order to fulfil autonomous ball collection. Script reads a file poses_new, converts it to a Python list and then moves to each of the points. It waits for the previous goal to be reached before proceeding to the next Pose goal.

**point_list.py** – not used anymore. I created this to test AMCL and navigation of the robot. As well as to workout points that robot has to navigate to in order to fulfil autonomous ball collection. Script reads a file poses_new, converts it to a Python list and then moves to each of the points. It waits for the previous goal to be reached also it waits for the user to press enter in the terminal before proceeding to the next Pose goal.

**tf_test.py** – not used anymore. I created this to test working out object position on the map from find_object_2d data. Parts of this code were reused in the main control script.

##### Package also contains the folder object2 which contains images of all known objects to recognise.

Package also contains folder find_object_2d_config which contains configuration of find_object_2d package (what algorithm to use etc).

## Malak_control

Package responsible for describing robot movement and robot state. It uses two controllers (that take care of actuations).

**joint_state_controller** – controls state of joints

**mobile_base_controller** – it uses differential drive controller to control the wheel joints. It is described in diff_drive.yaml file that contains configuration for the differential drive and actuations (e.g.limits)

#### Package has a single launch file:

**control.launch**  – the launch file is responsible for spawning the robot as well as its corresponding controllers. It is also spawning robot_state_publisher that takes care of transforms between various robot links. Inside the control.launch file there is a static transform between the base_link and the laser sensor. It is required by AMCL

## Malak_gui

This package is responsible for a full control of the robot. It contains two launch files that can either spawn the GUI on its own or perform a full system start (Gazebo, Obstacles, Robot, Controls, Nav Stack and GUI).

GUI is allowing manual, semi autonomous and fully autonomous control of the robot.

GUI was designed using PyQt Designer, an application that allows designing GUIs in Python effortlessly using a graphical interface rather than code.

There are two launch files:

**gui.launch** – Launches GUI on its own, this sets a GUI file as ros parameter which is used in a python script. This is required unlike in the other Python scripts as it is the only Python script that reads a file and whilst being run through a launch file.

**malak.launch** – this launch file performs a full system start. It starts assessment world, it spawns objects, it starts navigation stack (and the robot itself), it starts object recofgnition, it creates a transform between base_link and camera so objects are position correctly on the map and finally it starts main.py Python script which the program containing main logics of the robot.

#### Scripts:

**desing.py** – not used anymore, was used in an attempt of changing the design of the application to Python library, idea was dropped as it was making workflow inefficient.

**icons.py** – contains all the arrows used in manual operation mode. It is exported file from the PyQt designer and converted to python script using 3rd party software.

**main.py** – script containing GUI and all logics of the application

**res_rc.py** – also exported from PyQt Designer in attempt to make icons available in main python script, not used anymore.

## Malak_nav

Navigation package that utilises AMCL and move_base ROS packages. It is responsible for self-navigating the robot based on the given position.

#### It has three launch files:

**amcl.launch** – a launch file for AMCL package, it contains configuration of AMCL.

**move_base.launch** – a launch file for move_base package. This package allows controlling robot by sending targets aka goals. It uses two types of costmaps local and global. When robot is calculating paths for the given target it will use both of them to justify the best path. Robot is able to move backward and forward and it has 5 cms position tolerance and 0.17 radian orientation tolerance.

**malak_nav.launch** – a launch file that puts all of them together, it spawns the robot, starts a map server, starts amcl and starts move_base package. It forms a navigation stack of the robot. Navigation stack – everything that is required for the robot to move on its own – it know where it is and if you give it a goal it knows how to get there.

## Malakrobo_description

Package that contains the description of the robot as well as sensor description.

Folder called meshes, contains mesh files. Most of them are not used anymore (it is a leftover after previous iteration of the robot) – the only mesh used is rplidar.dae which contains visuals of the lidar used.

#### It contains two launch files:

**gazebo.launch** – it launches robot in gazebo

**display.launch** – it launches the robot in RViz

Description files:

**malakrobo.xacro** – main description file (URDF) of the robot that contains information how robot should look like. Most of the links have visual, collision and intertial parameters defined.

 - ***Visual*** - responsible for visual representation of the robot components
 - ***Collision*** - responsible for physical reperesentation of the robot components
 - ***Inertial*** – responsible for describing inertia (mass distribution) of the roboto components

The only exception from that are vacuum effectors – which do not have collision property defined. It is because vacuum effectors simulated in Gazebo use mechanism that checks if the center of the effector overlaps with the center of the object to be grasped.

**malakrobo.gazebo** – contains description of elements in Gazebo. It will contain various plugins to describe sensors and actuators in Gazebo. Components like differential drive can work only because they are using a plugin that describes interactions and behaviours of the certain component.

#### Components that are using external plugins:

 - ***Camera*** – uses libgazebo_ros_openni_kinect.so -> it includes both rgb and depth camera
 - ***Lidar*** – uses libgazebo_ros_laser.so
 - ***Vacuum effector (suction cup)*** - libgazebo_ros_vacuum_gripper.so
 - ***Differential drive*** - libgazebo_ros_diff_drive.so (differential drive allows two wheels to rotate independently around the same axis)
 - ***Gazebo ROS Control*** – uses libgazebo_ros_control.so 

## Malakrobo_mapping

Package responsible for generating the map for the robot to use with AMCL. 

#### It has three launch files.

**simulation.launch** – it starts RVIZ

**simulation_mapping.launch** – it starts slam gmapping and rviz. After map is completed it has to be saved. This launch file is not used anymore as it was for the old version of the laser scan sensor.

**lidar_mapping.lauch** – it starts slam gmapping and rviz. After map is complete the map has to be saved and then loaded into AMCL. This launch file is CURRENTLY being used for generating maps.

## Malakrobo_publisher

Contains a single launch file to start keyboard control of the robot.

Keyboard control (teleop_key.py) was implemented by me as I wanted to allow operator to control robot using keyboard or arrows in Manual operation mode in GUI. If I used existing package this would not be possible (especially that I have to switch it off when semi-autonomous or fully autonomous mode is on).

**key_control.launch** – starts teleop_key.py Python script that takes keyboard inputs to control the robot. The script publishes messages on cmd_vel topic depending on the key pressed by the user.

Scripts folder contains multiple bash scripts that start the robot. It was included in this package as at the time of writing it was the only package with scripts.

None of the bash scripts are required at this point as Malak_gui package starts the whole system automatically through a launch file.

#### Scripts:

**move2goal.py** – copied from turtlebot package for reference (I thought of implementing move 2 goal myself before finding out that there’s a package for it – move_base), it is not used by my robot

**pose.py** – it only displays the current odometry information – same thing as rostopic echo /odom. I did it while trying to understand publishing and subscribing in ROS. It is not used anymore.

# Main.py - functions documentation

**on_release(key)** – a function that is being called when the key is released it takes a single parameter – key that was released. It is setting the movement command depending on the key release to 0

**on_press(key)** – a function that is being called when the key is pressed it takes a single parameter – key that was pressed. It is setting the movement command depending on the key pressed to the corresponding velocity value.

**forward()** – if arrow up is pressed this function is called it sets forward linear velocity to 0.5

**forward_left()** – if arrow up-left is pressed this function is called it sets forward linear velocity to 0.5 and angular velocity to 0.5

**forward_right()** – if arrow up-left is pressed this function is called it sets forward linear velocity to 0.5 and angular velocity to -0.5

**backward()** – if arrow down is pressed this function is called it sets forward linear velocity to -0.5

**backward_left()** – if arrow down-left is pressed this function is called it sets forward linear velocity to -0.5 and angular velocity to -0.5

**backward_right()** – if arrow down-left is pressed this function is called it sets forward linear velocity to -0.5 and angular velocity to 0.5

**reset_fb()** – if up or down arrows are released it sets forward linear velocity to 0.0

**left()** – if arrow left is pressed this function is called it sets angular velocity to 0.5

**right()** – if arrow right is pressed this function is called it sets angular velocity to -0.5

**reset_lr()** – if left or right arrows are released it sets angular velocity to 0.0

**reset_both()** – calls both reset_fb and reset_lr functions to reset both forward linear velocity and angulary velocity when up-left, up-right, down-left or down-right arrows are released

**exit_handler()** – it is called when the X gui button is pressed (when application is closed). It stops all the threads and exits the program

**go_to_goal()** – uses move base simple to navigate to the goal – nothing is dependent on it so it does not require move_base action server that will give it feedback whether it reached it or not.

**take_ball_home()** – uses move_base topic to take ball to the goal. It is using action server as there are multiple steps to take the ball to the goal. When the target is reached suction cup is turned off to drop a ball.

**move_to_pose_single(client, x, y, z, w)** – function that moves the robot to a target position it takes 5 parameters, client which is an action client and x,y position as well as z, w orientation. X,y orientation don’t matter as the robot is incapable of tilting. Z position is not required as robot stays on the same level.

**get_ball_position(obj)** – function used to spawn a thread that transforms detected balls position in relation to map. It takes a single parameter, which is corresponding to the object that should be recognised. Multiple objects are used in find_object_2d package as I had to take pictures from different angles due to different lights and shading.

**calculate_pose_1()** – it works out if any of the balls’ position is within the given threshold for the current row. If it is, it sets ball’s position as the next target the robot should travel to pick it up. This function is for large ball.

**calculate_pose_2()** – it works out if any of the balls’ position is within the given threshold for the current row. If it is, it sets ball’s position as the next target the robot should travel to pick it up. This function is for medium ball.

**calculate_pose_3()** – it works out if any of the balls’ position is within the given threshold for the current row. If it is, it sets ball’s position as the next target the robot should travel to pick it up. This function is for small ball.

**row(pose_index)**  – MAIN AUTONOMOUS LOGICS!!! - Function that takes parameter pose_index which corresponds to the action given. If it is smaller than 10 it means there’s no ball to be picked up and it is just travelling to the given point. It is achieved through iterating through list of poses (dependant on the pose_index value). It will go to each of the poses until the list is exhausted.

If pose_index is greater than 10 it can only have 3 values, 10, 20 or 30 where each of them corresponds to small, medium or large ball respectively. Function will go through multiple points and it will try find the desired ball at each of the steps. Once the ball is found, it will set it as the next target and attempt to pick it up. If the ball is picked up it is going to take it back to goal.

Additionally, if full_sequence global variable is set to True. It will start with pose_index equal to 30 (large ball), after successfully completing the sequence(taking it back to the goal) it will recursively call itself with the next index(20), once it picks up the ball it will call itself recursively again with the next index(10) and attempt to find a small ball and take it home.

Function has lots of repetitive code as it was initially easier to do it this way, however when I was implementing the next stages I have realised it is not the best solution and I am looking to change it in the near future. Code here is really messy and hard to understand which is a really bad programming practice, but I am not really a great programmer, so it was hard to optimize it in the given period of time.

**go_to_row_1()** - callback function that navigates the robot to the start of row 1

**go_to_row_2()** - callback function that navigates the robot to the start of row 2

**go_to_row_3()**  - callback function that navigates the robot to the start of row 3

**leave_ball()** – callback function executed when take ball home button is pressed it navigates to the goal and it leaves the ball

**object_detection()** – callback function executed when Visual recognition button is pressed. It starts visual recognition and tries to navigate to objects

**control_suction()** – callback function executed when Turn on/off suction button is pressed. It sets the suction to the opposite state e.g. when it is on it will turn it off

**small_ball_auto()** – callback function executed when Pick up the small ball button is pressed. It spawns a row thread with parameter pose_index 10 (pick up the small ball in autonomous manner)

**medium_ball_auto()** – callback function executed when Pick up the medium ball button is pressed. It spawns a row thread with parameter pose_index 20 (pick up the medium ball in autonomous manner)

**large_ball_auto()** – callback function executed when Pick up the large ball button is pressed. It spawns a row thread with parameter pose_index 30 (pick up the small large in autonomous manner)

**auto_full_sequence()** – callback function executed when Start autonomous sequence button is pressed in Full autonomous operation mode. It will start row thread with parameter pose_index 30 and it will keep calling itself with the next index until all balls are in the goal

**amcl_window_update()** – callback function of AMCL Subscriber. It updates text for amcl label in GUI. It is running in a thread subscribed to amcl. It updates it every time a new pose arrives.

**goal_update()** – callback function of goal subscriber. It updates the current goal every time there’s a new message on the topic. It also globally assigns pose_count so every next goal can have a correct sequential number.

**gui_update()** – Function that updates different element of Graphical User Interface. It reads global variables and sets corresponding labels (all text in GUI is written in labels) accordingly.

**start_suction()** – vacuum effector works in the way that it gets activated when an empty message is sent to on or off service it will change state. This function activates all three vacuum effectors as in the physical model they would be one. In the simulated model it was necessary as vacuum effector is only grabbing an object if their centers are close to each other. Due to the size of the balls one vacuum effector wouldn’t be able to grab all of them (different center heights). This function sends a signal to on service of all three vacuum effectors.

**start_suction()** – vacuum effector works in the way that it gets activated when an empty message is sent to on or off service it will change state. This function deactivates all three vacuum effectors as in the physical model they would be one. In the simulated model it was necessary as vacuum effector is only grabbing an object if their centers are close to each other. Due to the size of the balls one vacuum effector wouldn’t be able to grab all of them (different center heights). This function sends a signal to off service of all three vacuum effectors.

**talker()** – a function that publishes messages on the cmd_vel topic. It has a function that check if the last message sent was 0.0 on all values it will stop sending messages. It is to prevent unnecessary communication as well as stopping the robot while it is performing move_base actions.

**auto_object_target()** – modified copy of action_controller script that was translated from C++. It is rotating itself so it faces the object (it is right in the middle of the camera). If it is in the middle, robot will additionally slowly start moving towards it. It is also able to perform recovery action – if detection is enabled (a global variable that controls if the robot should try to center itself towards the ball) – and there were no objects detected in the past 15 frames. The robot will attempt to rotate left and right to detect object. It is done as visual recognition does not work perfectly yet so sometimes it loses the target.

**lrg_ball_state_update()** – callback function for the large ball vacuum effector state subscriber. It will set lrg_ball_grabbed variable to true when the ball is grasped.

**med_ball_state_update ()** – callback function for the medium ball vacuum effector state subscriber. It will set med_ball_grabbed variable to true when the ball is grasped.

**sml_ball_state_update()** – callback function for the small ball vacuum effector state subscriber. It will set sml_ball_grabbed variable to true when the ball is grasped.

**watch_window_tab()** – function that monitors the currently selected tab in GUI. This is used to disabled manual controls when the semi or fully autonomous operation tabs are active.

**setup()**  – main function that starts all threads that update values, as well as all subscribers. It also defines a node system_controller. Finally, it starts GUI.
