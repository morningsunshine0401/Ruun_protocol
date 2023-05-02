#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include "ruun.h"
#include <ros/ros.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/VFR_HUD.h>
#include <arpa/inet.h>
#include <netinet/in.h>


using namespace std;
using namespace chrono;


Position global_position;
Heading vfr_hud;

void globalPositionCallback(const mavros_msgs::GlobalPositionTarget::ConstPtr& msg) {
  global_position.latitude = msg->latitude;
  global_position.longitude = msg->longitude;
  global_position.altitude = msg->altitude;
  global_position.vx = msg->velocity.x;
  global_position.vy = msg->velocity.y;
  global_position.vz = msg->velocity.z;
  //global_position.heading = 0;
}

void vfrHudCallback(const mavros_msgs::VFR_HUD::ConstPtr& msg) {
  vfr_hud.airspeed = msg->airspeed;
  vfr_hud.groundspeed = msg->groundspeed;
  vfr_hud.heading = msg->heading;
  vfr_hud.throttle = msg->throttle * 100; // Convert from ratio to percentage
  vfr_hud.altitude = msg->altitude;
  vfr_hud.climb_rate = msg->climb;

  //global_position.heading = msg->heading; // Update the heading in the global_position struct
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "mavros_to_ruun");
    ros::NodeHandle nh;

    ros::Subscriber global_position_sub = nh.subscribe<mavros_msgs::GlobalPositionTarget>(
        "mavros_msg/GlobalPositionTarget", 10, globalPositionCallback);
    ros::Subscriber vfr_hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>("mavros_msg/VFR_HUD", 10, vfrHudCallback);

  // Create a UDP socket
  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    return 1;
  }

  // Set up the socket address for the server
  struct sockaddr_in servaddr;
  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");  // IP address of the server
  servaddr.sin_port = htons(5000);  // port number of the server


  while (ros::ok()) {
  // Populate the Ruun_Message struct
  Ruun_Message msg;
  msg.position = global_position;
  msg.heading = vfr_hud;

  // Set the message header values
  msg.header.start_byte = 0xAB; // Arbitrary start byte value
  msg.header.payload_length = sizeof(Position) + sizeof(Heading);
  msg.header.message_sequence++; // Increment the message sequence number
  msg.header.system_id = 1; // Set your system ID
  msg.header.component_id = 1; // Set your component ID
  msg.header.message_id = 1; // Set your message ID

  // Compute the message checksum
  msg.checksum = crc16((double*)&msg, sizeof(msg) - sizeof(double));



  // Convert the message to a byte array
  double buf[sizeof(msg)];
  memcpy(buf, &msg, sizeof(msg));

  // Send the message to the server
  int n = sendto(sockfd, buf, sizeof(buf), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
  //int n = sendto(sockfd, encrypted_data, encrypted_data_length, 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
  ros::spinOnce();
}

  // Close the socket
  close(sockfd);

  return 0;
}

