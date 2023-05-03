# Ruun_protocol
The RUUN protocol Project aims to create a robust and efficient communication protocol between UAVs. This protocol allows for real-time data exchange and ensures data integrity through message encryption and checksums. The project leverages ROS (Robot Operating System) for handling the communication framework.
Project Overview

The project consists of the following components:

    ruun_protocol_pkg: This package contains the source code for the communication protocol implementation, which includes sending and receiving messages, as well as handling message integrity through checksums.

    ruun_crypto: This module provides encryption and decryption functionality for the messages exchanged between the robots and the control stations. It uses Pybind11 to interface between C++ and Python, allowing the use of Python-based encryption libraries.

Methods

    Socket Communication: The project uses UDP sockets for sending and receiving messages between the robot and the control station.

    Message Integrity: To ensure the integrity of the messages exchanged, a CRC-16 (Cyclic Redundancy Check) checksum is calculated and included in each message.

    Encryption and Decryption: The ruun_crypto module handles the encryption and decryption of messages using Python-based encryption libraries. This ensures secure communication between the robot and the control station.

Setting Up Your Workspace

To set up your workspace to use this project, follow the steps below:

    Clone the project repository into your ROS workspace's src directory:

    bash

cd ~/your_ros_workspace/src
git clone https://github.com/your_username/ruun_robotics_project.git

Replace your_ros_workspace with the path to your ROS workspace and your_username with your GitHub username.

Install the required dependencies:

csharp

sudo apt-get install ros-melodic-ros-base
sudo apt-get install python-pybind11

Replace melodic with your ROS distribution if you are using a different version. Also, make sure to install any additional dependencies mentioned in the package.xml files.

Build the project:

bash

cd ~/your_ros_workspace
catkin_make

Source the workspace:

bash

    source devel/setup.bash

    Run the necessary nodes and launch files as specified in the project documentation.

You are now ready to use the RUUN Robotics Project in your ROS workspace. For more information on how to use the project's components and integrate them into your robotic system, refer to the project's documentation and example implementations.
