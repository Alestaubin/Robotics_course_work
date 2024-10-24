#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>


ros::Publisher pub;

void call_back_function ( const std_msgs::String& msg) {
	//Do something with the topic that is listened
	
	geometry_msgs::Twist twist;
	std::string command = msg.data;
	
	twist.linear.x = twist.linear.y = twist.linear.z = 0;
	twist.angular.x = twist.angular.y = twist.angular.z = 0;

	if (command == "i") {
		//move forward
		twist.linear.x = 0.5;
	} else if (command == "u") {
		// turn left
		twist.linear.x = 0.5; 
		twist.angular.z = 0.5;	
	} else if (command == "o") {
		// turn right 
		twist.linear.x = 0.5;
		twist.angular.z = -0.5;
	}	
	pub.publish(twist);
}

int main(int argc, char **argv) {

	// initialize ROS
	ros::init(argc, argv, "subscriber_node") ;

	// create a node handle
	ros::NodeHandle node_handle ;

	// publisher
	pub = node_handle.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1) ;

	// specify the frequency to 100HZ
	ros::Rate loopRate(100) ;

	// listening to topic "/teleop/cmd" 
	ros::Subscriber subscriber = node_handle.subscribe("/teleop/cmd", 1, call_back_function) ;

	while ( ros::ok() ) {

		ros::spinOnce() ;

		loopRate.sleep() ;
	}

	return 0 ;
}

