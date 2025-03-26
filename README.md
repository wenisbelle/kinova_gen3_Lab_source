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

This file expects a joystick connected in the port /dev/input/js0 (more on that in the move servo section), so if you don't want to use this or it's a different port just comment this line. 







For now, I experienced some problems building all the system from the docker image, but I'll fix that on the near future. 




