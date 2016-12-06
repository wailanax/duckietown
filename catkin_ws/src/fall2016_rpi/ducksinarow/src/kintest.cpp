#include "ros/ros.h"
#include <string>
#include <ros/console.h>

#include <duckietown_msgs/Twist2DStamped.h>
#include <duckietown_msgs/WheelsCmdStamped.h>

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "mdkintest_node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh_;

  //ros::Subscriber sub_twist_;
  //ros::Publisher pub_wheelsCmd_;

  ros::Publisher pub_wheelsCmd_ = nh_.advertise<duckietown_msgs::WheelsCmdStamped>("wheels_cmd", 10);

  ros::Rate loop_rate(10); //10 Hz?;

  double Kleft = 25.0;
  double Kright = 25.0;
  double rleft = 0.02;
  double rright = 0.02;
  double bwidth = 0.1;
  
  while (ros::ok())
  {

    double omega_r = 5;
    double omega_l = 5;
    
    double rduty = omega_r / Kright;
    double lduty = omega_l / Kleft;

    duckietown_msgs::WheelsCmdStamped cmd_msg;

    cmd_msg.header.stamp = ros::Time::now();  //Keep the time the command was given to the wheels_driver
    cmd_msg.vel_left = float(lduty);
    cmd_msg.vel_right = float(rduty);
    pub_wheelsCmd_.publish(cmd_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}