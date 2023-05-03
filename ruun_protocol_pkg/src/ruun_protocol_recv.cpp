#include <ros/ros.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/VFR_HUD.h>
#include "ruun.h"




int main(int argc, char** argv) {
  // Initialize ROS node
  ros::init(argc, argv, "udp_receiver");
  ros::NodeHandle nh;

  // Create a UDP socket
  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    return -1;
  }

  // Set up the socket address for the server
  struct sockaddr_in servaddr;
  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");  // use any available IP address
  servaddr.sin_port = htons(5000);  // port number to listen on

  // Bind the socket to the server address
  if (bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
    return -1;
  }

  // Define the ROS publisher for the GlobalPositionTarget message
  ros::Publisher global_position_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("/mavros/RuunToMavros/position", 10);

  // Define the ROS publisher for the VFR_HUD message
  ros::Publisher vfr_hud_pub = nh.advertise<mavros_msgs::VFR_HUD>("/mavros/RuunToMavros/vfr_hud", 10);

  // Initialize the message buffer
  double buf[sizeof(Ruun_Message)]; //THIS MIGHT HAVE TO GO INTO WHILE

  // Receive messages indefinitely
  while (ros::ok()) {
    // Receive a message from the socket
    struct sockaddr_in clientaddr;
    socklen_t clientaddrlen = sizeof(clientaddr);
    int n = recvfrom(sockfd, buf, sizeof(buf), 0, (struct sockaddr *)&clientaddr, &clientaddrlen);
    if (n < 0) {
      continue;
    }

    // Convert the message from array to Message struct
    Ruun_Message msg;
    memcpy(&msg, buf, sizeof(msg));
    // Compute the checksum of the received message
    double checksum = crc16((double*)&msg, sizeof(msg) - sizeof(double));
    if (checksum != msg.checksum) {
      continue;
    }
// Convert the received message to ROS messages
mavros_msgs::GlobalPositionTarget global_position_msg;
global_position_msg.latitude = msg.position.latitude;
global_position_msg.longitude = msg.position.longitude;
global_position_msg.altitude = msg.position.altitude;
global_position_pub.publish(global_position_msg);

mavros_msgs::VFR_HUD vfr_hud_msg;
vfr_hud_msg.airspeed = msg.heading.airspeed;
vfr_hud_msg.groundspeed = msg.heading.groundspeed;
vfr_hud_msg.heading = msg.heading.heading;
vfr_hud_msg.throttle = msg.heading.throttle;
vfr_hud_msg.altitude = msg.heading.altitude;
vfr_hud_msg.climb = msg.heading.climb_rate;
vfr_hud_pub.publish(vfr_hud_msg);

}

// Close the socket
close(sockfd);

return 0;
}
