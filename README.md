# Kinova Gen3 7dof manipulator

This repository provides all the devolopements made by the SVTRP Team at CTEx with the Kinova Gen3 7dof manipulator. 


## Installation

The system runs inside a docker container, so be sure you have the docker engine installed. Then, change the cuda version inside dockerfile_nvidia_kinova_humble to match your system requirements and finally build the image:

    docker build -f dockerfile_nvidia_kinova_humble -t nvidia_ros_humble_kinova_test .

Get rest, because it will take a while to complete. In my computer it crashed the system when I tried just to build the system with colcon build, even decreasing the number of parall workers, so my solution was building one package at time with the following conditions

    export MAKEFLAGS="-j 1"
    colcon build --executor sequential

I'm going to change the name of the image in the near future, because it was primarly just a test building the system from source (more explanations on Move servo section), but now it's our standard project. 

Then you need to run the docker-compose file to create the container:

    docker-compose -f docker-compose.yml up

This file expects a joystick connected in the port /dev/input/js0 (more on that in the move servo section), so if you don't want to use this or it's a different port just comment/modify this line. 

Also, remember to give display permission for the docker:

    xhost local:

For now, I experienced some problems building all the system from the docker image, but I'll fix that on the near future.

## Basic Operation

I've created a ros2 package named main_launcher_pkg, whose goal is just regropup all launch files and applications inside one package. I think this makes simpler to run and organize the development, launching the packages with the configurations we want. 

A simple explanation for each launch file:

### Simulation
To simulate the robot using ign gazebo just launch:
    
    ros2 launch main_launcher_pkg simulation.launch.py

A table base was added to represente the physical table that the robot is mounted. For that, instead of implementing this in the world file, I changed the .xacro file to implement a link named table, because I wanted that the moveit package could also recognize this as an obstacle. 

If you want to remove this you can simple remove the following code from the gen3.xacro and kinova.urdf.xacro files inside kortex_description/robots directory.

    <link name="table">
        <inertial>
          <origin xyz="0 0 0" rpy="0 0 0" />
          <mass value="1.0" />
          <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />
        </inertial>
        <visual>
          <origin xyz="0 0 0" rpy="0 0 0" />
          <geometry>
            <box size="2 1 0.1" />
          </geometry>
        </visual>
        <collision>
          <origin xyz="0 0 0" rpy="0 0 0" />
          <geometry>The moveit package can be launched with:

            <box size="2 1 0.1" />
          </geometry>
        </collision>
    </link>

    <joint name="table_joint" type="fixed">
        <origin xyz="0.95 0 -0.05" rpy="0.0 0.0 0.0" />
          <parent link="world"/>
          <child link="table" />
    </joint>

With the previus system working, the moveit package can be launched with:

    ros2 launch main_launcher_pkg move_group_sim.launch.py

This command will launch the move_group node and rviz. This is the classical implementation of moveit2 with the configuration setup required for the simulation.

In order to launch the servo node, that enables the user to control the End Effector pose with the joystick just run (you need to run before the simulation.launch.py package):

    ros2 launch main_launcher_pkg moveit_servo_sim.launch.py

This launch file also launches RVIz, so depending on your application you may remove this. Also, the package required to run the joystick was joy_linux and not the more widely used joy. But the simple joy node didn't work inside the container, some modifications were required, so I just used the joy_linux that has the same results.

### Real Robot

To launch the real robot we can start with:

    ros2 launch main_launcher_pkg robot.launch.py

This will launch the robot interface with ros (controllers, topics and nodes).

Differently from the simulation, the following command will run the configurations of the robot with ROS and the moveit packages at the same time, so if you ran the previus command be sure you killed it before running the following command.

    ros2 launch main_launcher_pkg robot_move_group.launch.py

The main modification from the original file is the implementation of database with the most important manipulator positions and you can see in this node (ref: https://moveit.picknik.ai/main/doc/how_to_guides/persistent_scenes_and_states/persistent_scenes_and_states.html): 

        move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
        ],
    )

To manipulate the arm with the joystick, controlling the EOF pose, you need to run:

    ros2 launch main_launcher_pkg robot.launch.py

    ros2 launch main_launcher_pkg robot_servo.launch.py

This package was the most critical by far, because movement was extremely shaky. This is a well documented problem that happens when the manipulator doesn't have an internal filter, which is the case here. This issue was already addressed and resolved in this pull request (https://github.com/moveit/moveit2/pull/2594), but unfortunatelly, this change was implemented only in ros2 jazzy and we need the ros humble to work with the vision package. So, I got four options:
    
1 - Passing everything to jazzy, including the vision: This one is not feasible, because the vision module could not work properly and this is a crucial package for our system. 


2 - Passing the move_group nodes to jazzy and maintain the vision on humble, using two different containers communicating between then. With my current knowlodge of Docker I think it's possible, but I was afraid that this could bring some communications issues with the arm. So I tried another solution before jumping into this. 

3 - Implemente the modification proposed in the pull request above on humble. Although this is the best option technically speaking, the architectury of the package changed consideraly from humble to jazzy, so it would require a lot of work. It was discarted for the moment but it's a long stand goal for this project. 

4 - Trying to implement velocity control based in this article from black coffe robotics (https://www.blackcoffeerobotics.com/blog/ros-moveit-servo-with-kinova-robot-arm): " As a simple circumvention to this problem, we found the response from the velocity controller to be more reliable in simulation, and the physical rob". 

I choose the fourth possibility, because I already have experience in dealing with ros2 controls and it looked like the simplest solution. That's the reason this repository is build from source, to change the controllers inside the *ros2_controllers.yaml* file. Perhaps the best solution is not the implemented, because it still used the *joint_trajectory_controller*, but it worked pretty nicely. 

### Vision System

Just run:
    
    ros2 launch main_launcher_pkg robot_vision.launch.py

## Modification from the source code

### Adding the table simulation and to gen3.xacro, visualized in moveit

The arm is installed on top of a table in the laboratory, so I changed the urdf files to represent this configuration on moveit and also in the gazebo simulation. That was a simple changed but in depending on the configuration it's necessary to modify the following code from the gen3.xacro and kinova.urdf.xacro files inside the /kortex_description/robots directory.

    <link name="table">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="1.0" />
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="2 1 0.1" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="2 1 0.1" />
      </geometry>
    </collision>
  </link>

  <joint name="table_joint" type="fixed">
    <origin xyz="0.95 0 -0.05" rpy="0.0 0.0 0.0" />
      <parent link="world"/>
      <child link="table" />
  </joint>

ros2 launch main_launcher_pkg robot_vision.launch.py

## Moveit2_scripts

This package has some c++ scripts testing some automations on the movement of the robot. For now I don't have a clearly goal with that but it can be integrated with the vision system in the future. 

## Yolo
This is also very experimental, for now It only gets the image and draw a bounding box around people. 


## To do

[ ] Implement the move servo with mediapipe 

[ ] Implement the move servo with the imu sensor 

[ ] Creating our own classification dataset of mortar ammunitions and train in yolo

[ ] Implementing the automatic detection with the visual system 

