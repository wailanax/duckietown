#include "ros/ros.h"
#include <string>
#include <ros/console.h>

#include <duckietown_msgs/Twist2DStamped.h>
#include <duckietown_msgs/BoolStamped.h>
#include <duckietown_msgs/Pose2DStamped.h>
#include <duckietown_msgs/WheelsCmdStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

using namespace std;

class ducksinarow_node
{
public:
ducksinarow_node(); // constructor

// class variables_
private:
  ros::NodeHandle nh_; // interface to this node
  string node_name_;
  string veh_name_;

  ros::Publisher pub_wheelsCmd_;

  ros::Subscriber sub_vehicleVelocity_;
  ros::Subscriber sub_vehiclePose_;


  double V_; // velocity
  double Omega_; // omega
  double x_; // x position
  double y_; // y position
  double theta_; // current theta

  ros::Time previousTimestamp_;
  duckietown_msgs::Pose2DStamped odomPose_;

  // TODO camera functions with AR tags

  //kinematic functions

  void sendCmd(double v, double omega) {
  	duckietown_msgs::Twist2DStamped car_cmd_msg;
  	car_cmd_msg.header.stamp = ros::Time::now();
  	car_cmd_msg.v = v;
  	car_cmd_msg.omega = omega;

  	pub_wheelsCmd_.publish(car_cmd_msg);
  }

  void sendStop(double v, double omega) {
  	duckietown_msgs::Twist2DStamped car_cmd_msg;
  	car_cmd_msg.header.stamp = ros::Time::now();
  	car_cmd_msg.v = v;
  	car_cmd_msg.omega = omega;

  	pub_wheelsCmd_.publish(car_cmd_msg);
  }

  //callback functions

  void callbackVel(duckietown_msgs::Twist2DStampedConstPtr const& vel) {
  	ros::Time nextTimestamp = vel->header.stamp;
  	V_ = vel->v;
  	Omega_ = vel->omega;
  	
  }

  void callbackPos(duckietown_msgs::Pose2DStampedConstPtr const& pose) {
  	x_ = pose->x;
  	y_ = pose->y;
  	theta_ = pose->theta;
  	ROS_INFO_STREAM("position: " << x_ << ", " << y_ << ", " << theta_);
  }
  
};

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "ducksinarow_node");

  ducksinarow_node node;
  ros::spin();


  return 0;
}


ducksinarow_node::ducksinarow_node(): nh_("~"), node_name_("ducksinarow_node")
{

	//set initial params to 0
	nh_.param("V", V_, 0.0);
    nh_.param("Omega", Omega_, 0.0);
	nh_.param("x", x_, 0.0);
	nh_.param("y", y_, 0.0);
  	nh_.param("theta", theta_, 0.0);

	//Get the vehicle name
  	veh_name_ = ros::this_node::getNamespace();
  	veh_name_.erase(std::remove(veh_name_.begin(),veh_name_.end(), '/'), veh_name_.end());
  	ROS_INFO_STREAM("[" << node_name_<< "] Vehicle Name: " << veh_name_);


  	//Setup the publishers and subscirbers
	pub_wheelsCmd_ = nh_.advertise<duckietown_msgs::WheelsCmdStamped>("wheels_cmd", 10);

  	sub_vehicleVelocity_ = nh_.subscribe("car_cmd", 10, &ducksinarow_node::callbackVel, this);
  	
 	sub_vehiclePose_ = nh_.subscribe("pose", 10, &ducksinarow_node::callbackPos, this);


  ROS_INFO_STREAM("[" << node_name_ << "] has started.");
}