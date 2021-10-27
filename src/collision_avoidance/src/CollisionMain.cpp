#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_utils_fd.h"
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <sstream>

geometry_msgs::Twist vel_utente ;

geometry_msgs::Twist vel_corrette ;

bool coord_arrived ;

ros::Publisher vel_pub ;

void LetturaVel(const geometry_msgs::Twist::ConstPtr& msg) {
	ROS_INFO("Sto ricevendo le coordinate di dove l'utente desidera far andare il robot: x:%f y:%f z:%f", msg->linear.x, msg->linear.y, msg->angular.z);
	coord_arrived = true ;
	vel_utente.linear.x = msg->linear.x;
	vel_utente.linear.y = msg->linear.y;
	vel_utente.angular.z = msg->angular.z; 
}

void LaserCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
	if ( coord_arrived == false ) return ;
	coord_arrived = false ;// così da evitare di processare due volte uno stesso comando di velocità
	//inizializzazione oggetti necessari per la procedura
	laser_geometry::LaserProjection projector;
	tf::TransformListener listener;
	tf::StampedTransform transform ;
	
	
	/*if (!listener_.waitForTransform(scan_in->header.frame_id,"/base_link",
	scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
	ros::Duration(1.0))) return ;*/
	sensor_msgs::PointCloud cloud;
	try {
		//otteniamo punti ostacolo dal laser scanner
		projector.transformLaserScanToPointCloud("base_laser_link",*scan_in,cloud,listener);
		//otteniamo la trasformata per ottenere i punti ostacolo rispetto al robot
		listener.waitForTransform("base_link","base_laser_link",ros::Time(0),ros::Duration(10.0));
		listener.lookupTransform("base_link","base_laser_link",ros::Time(0),transform);
	}
	catch ( tf::TransformException& e ) {
		//ROS_ERROR("%s", e.what());
		std::cout << e.what();
		ros::Duration(1.0).sleep();
		return;
	}
	
	
	//otteniamo la trasformata sotto forma di matrice
	Eigen::Isometry2f transform_laser = convertPose2D(transform);
	
	Eigen::Vector2f p;
	float sum_x = 0;
	float sum_y = 0;
	for(auto& point :cloud.points) {
		if ( vel_utente.linear.x != 0 ) {
			p(0) = point.x ;
			p(1) = point.y ;
			
			p = transform_laser*p; //in questo modo ottengo il punto trasformato
			
			float modulo = 1/sqrt(point.x*point.x + point.y*point.y);
			
			sum_x-= p(0)*modulo*modulo;//p(0) / (p(0)*p(0) + p(1)*p(1));
			sum_y-= p(1)*modulo*modulo;//p(1) / (p(0)*p(0) + p(1)*p(1));
			//sottraggo invece di sommare perchè prendo l'opposto delle forze calcolate
			//questo perchè le forze che consideriamo hanno la stessa direzione e verso
			//opposto dei vettori robot->punto-collisione
		}
		else break;
	}
	ROS_INFO("sum_x è pari a: %f \n\n", sum_x);
	ROS_INFO("sum_y è pari a: %f n\n", sum_y);
	
	//setto le coordinate della velocità da passare al cmd_vel
	vel_corrette.linear.x = sum_x/500 + vel_utente.linear.x ;
	vel_corrette.angular.z = sum_y/800 + vel_utente.angular.z ;
	
	vel_pub.publish(vel_corrette);
	
	vel_corrette.linear.x = 0 ;
	
	vel_corrette.linear.y = 0 ;
	
	vel_corrette.angular.z = 0 ;
	
}
	


int main(int argc, char **argv) {
	
	ros::init(argc, argv, "CollisionMain");
	
	ros::NodeHandle n;
	
	ros::Rate loop_rate(10);
	
	ros::Subscriber vel_sub = n.subscribe("/cmd_vel_AUX",1000,LetturaVel); //leggo le coordinate date dall'utente sul topic cmd_vel_AUX
	
	ros::Subscriber vel_correct = n.subscribe("/base_scan",1000,LaserCallBack); //leggo il laserscanner e modifico le coordinate per far muovere il robot nella direzione corretta
																					// ( = in modo tale che non vada a sbattere )
		
	vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000); //scrivo sul topico cmd_vel le coordinate corrette
		
	ros::spin();
	loop_rate.sleep();
	
	return 0;
}

