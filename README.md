# Ruun protocol
<img src = "https://user-images.githubusercontent.com/109836008/235913879-c722f793-3a12-4aeb-97f1-bcfe06e1fd46.png" width="40%" height="40%">

The RUUN protocol Project aims to create a robust and efficient communication protocol between UAVs. This protocol allows for real-time data exchange and ensures data integrity through message encryption and checksums. The project leverages ROS (Robot Operating System) for handling the communication framework.

# 1. Project Overview

The project consists of the following components:

+ruun_protocol_pkg: This package contains the source code for the communication protocol implementation, which includes sending and receiving messages, as well as handling message integrity through checksums.
    ++ ruun_protocol.cpp: This file contains code for ROS to subscribe to topics from          PX4, convert the subscribed data into a Ruun_Message format, encrypt it, and then        send the message using UDP.
    
    ++ ruun_protocol_recv.cpp: This file contains code for receiving encrypted                                          Ruun_Message instances via UDP, decrypting them, and then                                publishing the corresponding MAVROS message topics using                                ROS.
    
    ++ ruun.h: A header file that defines the Ruun_Message format used in the Ruun                      protocol.
    
    ++ DoubleXORCipher.h: A header file that provides functions for encrypting and                                 decrypting messages using the Double XOR Cipher algorithm.



# 2. Methods

1. Socket Communication: The project uses UDP sockets for sending and receiving messages between the UAVs.

2. Message Integrity: To ensure the integrity of the messages exchanged, a CRC-16 (Cyclic Redundancy Check) checksum is calculated and included in each message.

3. Encryption and Decryption: The ruun_crypto module handles the encryption and decryption of messages using a simple XOR calculation. This ensures secure communication between the UAVs.

# 3. Setting Up Your Workspace

To set up your workspace to use this project, follow the steps below:

1. Clone the project repository into your ROS workspace's src directory:
    cd ~/ruun_ws/src
    git clone https://github.com/your_username/ruun_robotics_project.git

Replace your_ros_workspace with the path to your ROS workspace and your_username with your GitHub username.

2. Install the required dependencies:

    sudo apt-get install ros-melodic-ros-base

Replace melodic with your ROS distribution if you are using a different version. Also, make sure to install any additional dependencies mentioned in the package.xml files.

3. Build the project:

    cd ~/ruun_ws
    catkin_make

4. Source the workspace:

    source devel/setup.bash
    
5. Run the necessary nodes and launch files as specified in the project documentation.

You are now ready to use the RUUN Robotics Project in your ROS workspace. For more information on how to use the project's components and integrate them into your robotic system, refer to the project's documentation and example implementations.
