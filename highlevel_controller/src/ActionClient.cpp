#include <highlevel_controller/ActionClient.hpp>

ActionClient::ActionClient(ros::NodeHandle& node_handle) : 	node_handle_(node_handle),
															pose_command_action_client_ ("/gen3/action_planner/pose", true) {
	ROS_INFO("constructor"); 
	//initialize variables
	if ( !readParameters() ) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}

	// prepare all publishers, subscribers, servers, clients, etc
	
	ROS_INFO_STREAM("Successfully launched node.") ;
}

ActionClient::~ActionClient() {

}

bool ActionClient::readParameters() {
	ROS_INFO("ActionClient - readParameters()") ;

	if ( !node_handle_.getParam("/action_list/number_of_targets", NUM_TARGETS_)) {
		ROS_INFO("Cannot read parameter /action_list/number_of_targets" );
		return false ;
	}
	//initialize matrices
	target_translation_ = Eigen::MatrixXd::Zero(NUM_TARGETS_,3);
	target_orientation_ = Eigen::MatrixXd::Zero(NUM_TARGETS_,3);
	target_duration_ = Eigen::VectorXd::Zero(NUM_TARGETS_);

	for (int i = 0; i < NUM_TARGETS_; i++) {
		//read translation, orientation, and duration
		if (!node_handle_.getParam("/action_list/action_" + std::to_string(i) + "/translation", translation_row)) {
			//ROS_INFO("Cannot read parameter /action_list/action_" + std::to_string(i) + "/translation");
			return false;
		}

		if (!node_handle_.getParam("/action_list/action_" + std::to_string(i) + "/orientation", orientation_row)) {
			//ROS_INFO("Cannot read parameter /action_list/action_" + std::to_string(i) + "/orientation");
			return false;
		}

		if (!node_handle_.getParam("/action_list/action_" + std::to_string(i) + "/duration", duration)) {
			//ROS_INFO("Cannot read parameter /action_list/action_" + std::to_string(i) + "/duration");
			return false;
		}
		//push rows to the end of the matrix
		ROS_INFO("translation_row: %f, %f, %f  ", translation_row[0], translation_row[1], translation_row[2]);
		ROS_INFO("orientation_row: %f, %f, %f  ", orientation_row[0], orientation_row[1], orientation_row[2]);
		
		target_translation_(i,0) = translation_row[0];
		target_translation_(i,1) = translation_row[1];
		target_translation_(i,2) = translation_row[2];
		target_orientation_(i,0) = orientation_row[0];
		target_orientation_(i,1) = orientation_row[1];
		target_orientation_(i,2) = orientation_row[2];
		target_duration_(i) = duration;
	}
	return true;
}

void ActionClient::update() {

	ROS_INFO_STREAM("[ActionClient::update] Action client is ready. Wait for the server...") ;

	//boost::function fun_done = boost::bind(&ActionClient::doneCallback, this, _1, _2) ;

	for ( int counter = 0 ; counter < NUM_TARGETS_ ; counter++ ) {

		// Fill in goal here
		action_goal_.x = target_translation_(counter,0) ;
		action_goal_.y = target_translation_(counter,1) ;
		action_goal_.z = target_translation_(counter,2) ;
		action_goal_.roll = target_orientation_(counter,0) ;
		action_goal_.pitch = target_orientation_(counter,1) ;
		action_goal_.yaw = target_orientation_(counter,2) ;
		action_goal_.T = target_duration_(counter) ;

		// wait for the server
		pose_command_action_client_.waitForServer();

		pose_command_action_client_.sendGoal(action_goal_,
				boost::bind(&ActionClient::doneCallback, this, _1, _2),
				boost::bind(&ActionClient::activeCallback, this),
				boost::bind(&ActionClient::feedbackCallback, this, _1) ) ;

		ROS_INFO_STREAM("[ActionClient::update] Sent action goal. Waiting for the results...") ;
		pose_command_action_client_.waitForResult( ros::Duration(30.0) );

		if ( pose_command_action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) {
			if (pos_done && ori_done){
				ROS_INFO("[ActionClient::update] Yay! The end-effector reaches the goal position") ;
			}else if (!pos_done){
				ROS_ERROR("[ActionClient::update] End-effector position is not reached") ;
			}else if (!ori_done){
				ROS_ERROR("[ActionClient::update] End-effector orientation is not reached") ;
			}
		}
	}
	ROS_INFO_STREAM("[ActionClient::doneCallback] Done, shutdown") ;
	ros::shutdown();
}


void ActionClient::doneCallback(const actionlib::SimpleClientGoalState& state, const highlevel_msgs::PoseCommandResultConstPtr& result) {
	ROS_INFO("[ActionClient::doneCallback] Finished in state [%s]", state.toString().c_str());
	ROS_INFO("[ActionClient::doneCallback] Message: %s", result->message.c_str());

}

void ActionClient::activeCallback() {
	ROS_INFO_STREAM("[ActionClient::activeCallback] Action has become active");
}

void ActionClient::feedbackCallback(const highlevel_msgs::PoseCommandFeedbackConstPtr& feedback) {
	if (feedback->distance_translation < 0.1) {
		pos_done = true;
	}else{
		pos_done = false;
	
	}
	if (feedback->distance_orientation < 0.1) {
		ori_done = true;
	}else{
		ori_done = false;
	}
	if (j >= 10){
		ROS_INFO("[ActionClient::feedbackCallback] distance_translation: %f", feedback->distance_translation) ;
		ROS_INFO("[ActionClient::feedbackCallback] distance_orientation: %f", feedback->distance_orientation) ; 
		ROS_INFO("[ActionClient::feedbackCallback] time_elapsed: %f", feedback->time_elapsed) ;
		j = 0;
	}j ++;
}

int main(int argc, char** argv) {

	ROS_INFO("Client - main()") ;
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