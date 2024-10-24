#include <highlevel_controller/ActionClient.hpp>

int main(int argc, char** argv) {

	ROS_INFO("here... main()") ;
	ros::init(argc, argv, "pose_command_action_client") ;
	ros::NodeHandle node_handle ;

	ActionClient pose_command_action_client(node_handle) ;

	ros::Rate loop_rate(500) ;

	while ( ros::ok() ) {
		ros::spinOnce();
		pose_command_action_client.update() ;
		loop_rate.sleep() ;
	}

	return 0;
}
