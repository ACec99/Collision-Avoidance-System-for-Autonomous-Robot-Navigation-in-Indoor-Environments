#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
//#include "laser_geometry/laser_geometry.h"
//#include <Eigen/StdVector>
//#include <Eigen/Geometry>
#include <sstream>

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "CollisionMain");
	
	ros::NodeHandle n;
	
	ros::Subscriber vel_sub = n.subscribe("/cmd_vel_AUX",1000,LetturaVel);
	
	ros::Subscriber vel_correct = n.subscriber("/base_scan",1000,LaserCallBack);
	
	ros::Publisher vel_pub = n.advertise<geometry_msg::Twist>("/cmd_vel",1000);
	
	ros::spin();
	
	return 0;
}

