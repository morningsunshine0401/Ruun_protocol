#include <ros/ros.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/VFR_HUD.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "fixed_data_publisher");
  ros::NodeHandle nh;

  ros::Publisher global_position_pub =
      nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros_msg/GlobalPositionTarget", 10);
  ros::Publisher vfr_hud_pub = nh.advertise<mavros_msgs::VFR_HUD>("mavros_msg/VFR_HUD", 10);

  ros::Rate loop_rate(10); // 10 Hz

  double initial_latitude = 37.7749;
  double initial_longitude = -122.4194;

  while (ros::ok()) {
    mavros_msgs::GlobalPositionTarget global_position;
    global_position.latitude = initial_latitude;
    global_position.longitude = initial_longitude;
    global_position.altitude = 500;
    global_position.velocity.x = 1;
    global_position.velocity.y = 1;
    global_position.velocity.z = 0;

    initial_latitude += 0.0001; // Increment latitude by a small value
    initial_longitude += 0.0001; // Increment longitude by a small value

    // Wrap latitude and longitude values within the valid range
    initial_latitude = fmod(initial_latitude + 90, 180) - 90;
    initial_longitude = fmod(initial_longitude + 180, 360) - 180;

    mavros_msgs::VFR_HUD vfr_hud;
    vfr_hud.airspeed = 15.5;
    vfr_hud.groundspeed = 20.0;
    vfr_hud.heading = 10;
    vfr_hud.throttle = 0.5; // 50% throttle
    vfr_hud.altitude = 450.0;
    vfr_hud.climb = 2.0;

    global_position_pub.publish(global_position);
    vfr_hud_pub.publish(vfr_hud);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}