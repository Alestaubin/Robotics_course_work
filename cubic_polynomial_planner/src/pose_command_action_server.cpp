#include <pose_command_action_server/ActionServer.hpp>

ActionServer::ActionServer(ros::NodeHandle& node_handle) : 	node_handle_(node_handle),
															move_to_action_server_ (node_handle_, "/gen3/action_planner/pose", boost::bind(&ActionServer::move_to_ballback, this, _1 ), false),
															loop_rate_(500)
															{
														//	move_to_action_server_ (node_handle_, "go_to_home_configuration", false) {
	/*if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
		ros::console::notifyLoggerLevelsChanged();
	}*/
    ROS_INFO("server constructor") ;

    if (!node_handle_.getParam("/gen3/urdf_file_name", urdf_file_name)){
        ROS_INFO("Cannot read parameter /gen3/urdf_file_name, set to default value" );
    }
    if ( !node_handle_.getParam("/gen3/linear/max_velocity", lin_max_vel)) {
        ROS_INFO("Cannot read parameter /gen3/linear/max_velocity, set to default value" );
    }

    //build urdf model
    pinocchio::urdf::buildModel(urdf_file_name, model, false);
    pinocchio::Data data(model);
    data_ = data;

    //initialize values
    first_call = start_flag = false;
    dt = 0.002;
    elapsed_time = start_time = curr_time = total_time = 0;

    ref_EE_vel = Eigen::VectorXd::Zero(3);
    ref_EE_pos = Eigen::VectorXd::Zero(3);

    target_EE_pos = Eigen::VectorXd::Zero(3);
    start_EE_pos = Eigen::VectorXd::Zero(3);

    fbk_joint_vel = Eigen::VectorXd::Zero(7);
    fbk_joint_pos = Eigen::VectorXd::Zero(7);
    
    fbk_EE_pos = Eigen::VectorXd::Zero(3);
    fbk_EE_Rmatrix = Eigen::Matrix3d::Zero(3,3);
    fbk_EE_euler = Eigen::VectorXd::Zero(3);

    ref_EE_Rmatrix = Eigen::Matrix3d::Zero(3,3);
    ref_EE_euler = Eigen::VectorXd::Zero(3);

    fbk_EE_quat = Eigen::Quaterniond::Identity();
    q_ini = Eigen::Quaterniond::Identity();
    q_tar = Eigen::Quaterniond::Identity();
    qs = Eigen::Quaterniond::Identity();

    se3_ref_EE_pose = pinocchio::SE3::Identity();
    se3_motion_EE = pinocchio::SE3::Identity();
    motion_vec_local = Eigen::VectorXd::Zero(6);
    motion_vec_world = Eigen::VectorXd::Zero(6);

    ROS_DEBUG_STREAM("Variables initialized - server");

	feedback_subscriber_ =  node_handle.subscribe("/gen3/joint_states",1,&ActionServer::feedbackCallback, this);
    pose_publisher_ = node_handle.advertise<geometry_msgs::Pose>("/gen3/reference/pose", 1);
    twist_publisher_ = node_handle.advertise<geometry_msgs::Twist>("/gen3/reference/twist", 1);

	move_to_action_server_.start();

	ROS_INFO_STREAM("[ActionServer::ActionServer] action server is ready");

}

ActionServer::~ActionServer() {

}


void ActionServer::feedbackCallback(const sensor_msgs::JointState::ConstPtr& msg) {
	if ( move_to_action_server_.isActive() ) {
        for (i = 0; i < 7; i++){
            fbk_joint_pos(i) = msg->position[i];
            fbk_joint_vel(i) = msg->velocity[i];
        }

        pinocchio::forwardKinematics(model, data_, fbk_joint_pos, fbk_joint_vel);

        fbk_EE_pos(0) = data_.oMi[7].translation()(0);
        fbk_EE_pos(1) = data_.oMi[7].translation()(1);
        fbk_EE_pos(2) = data_.oMi[7].translation()(2);

        fbk_EE_Rmatrix = data_.oMi[7].rotation(); //fbk rotation matrix
        pinocchio::quaternion::assignQuaternion(fbk_EE_quat, fbk_EE_Rmatrix); //convert rotation matrix to quaternion

        fbk_EE_euler = pinocchio::rpy::matrixToRpy(fbk_EE_Rmatrix); //convert rotation matrix to euler angles

	}
}
void ActionServer::print_counter(){
    if (counter == 100){
        ROS_INFO("[ActionServer] elapsed_time: %f", elapsed_time);
        ROS_INFO("[ActionServer] target_EE_pos: %f, %f, %f", target_EE_pos(0), target_EE_pos(1), target_EE_pos(2));
        ROS_INFO("[ActionServer] ref_EE_pos: %f, %f, %f", ref_EE_pos(0), ref_EE_pos(1), ref_EE_pos(2));
        ROS_INFO("[ActionServer] fbk_EE_pos: %f, %f, %f", fbk_EE_pos(0), fbk_EE_pos(1), fbk_EE_pos(2));
        ROS_INFO("[ActionServer] ref_EE_euler: %f, %f, %f", ref_EE_euler(0), ref_EE_euler(1), ref_EE_euler(2));
        ROS_INFO("[ActionServer] fbk_EE_euler: %f, %f, %f", fbk_EE_euler(0), fbk_EE_euler(1), fbk_EE_euler(2));
        counter = 0;
    }
    counter++;
}
void ActionServer::update() {
    print_counter();
    if (!start_flag){
        return;
    }

    curr_time = ros::Time::now().toSec();

    if (first_call){
        start_time = curr_time;
        start_EE_pos = fbk_EE_pos; 
        q_ini = fbk_EE_quat;
        first_call = false; 
    }

    elapsed_time = curr_time - start_time;

    if(elapsed_time >= total_time){
        //start_flag = false;
        elapsed_time = total_time;
    }

    interpolation_parameter_dot = (6*elapsed_time)/pow(total_time, 2)-(6*pow(elapsed_time, 2))/pow(total_time, 3);
    interpolation_parameter = 3*pow(elapsed_time, 2)/pow(total_time, 2)-(2*pow(elapsed_time,3))/pow(total_time,3);
    
    //set feedback parameters
    pose_command_feedback_.distance_translation = (target_EE_pos - fbk_EE_pos).norm();
    pose_command_feedback_.distance_orientation = fbk_EE_quat.angularDistance(q_tar);
    pose_command_feedback_.time_elapsed = elapsed_time;

    //compute next EE velocity 
    planner_EE_vel();
    //compute next EE orientation
    planner_EE_ori();
    
    publish_topics();
}

void ActionServer::planner_EE_vel(){
    //ROS_INFO("server planner_EE_vel") ;
    //calculate next EE pose
    ref_EE_pos = start_EE_pos + interpolation_parameter*(target_EE_pos - start_EE_pos);
    //calculate next EE velocity
    // ref_EE_vel = (ref_EE_pos - fbk_EE_pos)*c_cubic;

    ref_EE_vel = (target_EE_pos - start_EE_pos)*interpolation_parameter_dot;

    //normalize velocity
    if (ref_EE_vel.norm() > lin_max_vel){
        ref_EE_vel = ref_EE_vel.normalized()*lin_max_vel;
    }

    //set next position in the transformation matrix
    /*
    se3_ref_EE_pose.translation() = ref_EE_pos;
    motion_vec_world.head(3) = ref_EE_vel;
    */
}

void ActionServer::planner_EE_ori(){
    //ROS_INFO("server planner_EE_ori") ;
    //calculate slerp
    qs = q_ini.slerp(interpolation_parameter, q_tar);

    //do the following calculations in inverse_kinematics_controller instead
/*
    ref_EE_Rmatrix = qs.toRotationMatrix(); //convert quaternion to rotation matrix  
    //set next orientation in the transformation matrix
    se3_ref_EE_pose.rotation() = ref_EE_Rmatrix;
    ref_EE_euler = pinocchio::rpy::matrixToRpy(ref_EE_Rmatrix); //convert rotation matrix to euler angles
    //compute next EE twist SE3 transformation matrix
    se3_motion_EE = data_.oMi[7].inverse()*se3_ref_EE_pose;
    //convert SE3 transformation matrix to twist vector
    motion_vec_local = pinocchio::log6(se3_motion_EE).toVector();
    //convert twist vector to world frame
    motion_vec_world.tail(3) = c_omega*data_.oMi[7].rotation()*motion_vec_local.tail(3);
    //motion_vec_world is the next EE twist in world frame
*/
}

void ActionServer::publish_topics(){
    //ROS_INFO("server publish_topics") ;
    //publish pose
    pose_msg.position.x = ref_EE_pos(0);
    pose_msg.position.y = ref_EE_pos(1);
    pose_msg.position.z = ref_EE_pos(2);
    pose_msg.orientation.x = qs.x();
    pose_msg.orientation.y = qs.y();
    pose_msg.orientation.z = qs.z();
    pose_msg.orientation.w = qs.w();
    pose_publisher_.publish(pose_msg);

    //publish twist
    twist_msg.linear.x = ref_EE_vel(0);
    twist_msg.linear.y = ref_EE_vel(1);
    twist_msg.linear.z = ref_EE_vel(2);
    twist_msg.angular.x = 0;
    twist_msg.angular.y = 0;
    twist_msg.angular.z = 0;

    /*twist_msg.angular.x = motion_vec_world(3);
    twist_msg.angular.y = motion_vec_world(4);
    twist_msg.angular.z = motion_vec_world(5); */
    twist_publisher_.publish(twist_msg);
}

void ActionServer::move_to_ballback(const highlevel_msgs::PoseCommandGoalConstPtr& goal) {
    ROS_INFO("Server received goal: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f, T=%f", goal->x, goal->y, goal->z, goal->roll, goal->pitch, goal->yaw, goal->T) ;
    //save goal Paramaters
    target_EE_pos(0) = goal->x;
    target_EE_pos(1) = goal->y;
    target_EE_pos(2) = goal->z;
    
    pinocchio::quaternion::assignQuaternion(q_tar, pinocchio::rpy::rpyToMatrix(goal->roll, goal->pitch, goal->yaw));

    total_time = goal->T;
    
    //set start flag
    start_flag = true;
    first_call = true;
    elapsed_time = 0;

    while(pose_command_feedback_.distance_translation > 0.1 || pose_command_feedback_.distance_orientation > 0.1 || elapsed_time < total_time){
		// publish the feedback. the client will be able to read it
		move_to_action_server_.publishFeedback(pose_command_feedback_) ;

		loop_rate_.sleep() ;
	}
    move_to_action_server_.publishFeedback(pose_command_feedback_) ;
	loop_rate_.sleep() ;

    pose_command_result_.message = "Done" ;
	move_to_action_server_.setSucceeded(pose_command_result_) ;

	ROS_INFO("[ActionServer::pose_command_ballback] Action is succesfully done, distance_orientation: %f", pose_command_feedback_.distance_orientation) ;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "action_server");
	ros::NodeHandle node_handle ;

	ActionServer action_server (node_handle) ;

	ros::Rate loop_rate (500) ;
	while ( ros::ok() ) {
		ros::spinOnce();
		action_server.update() ;
		loop_rate.sleep() ;
	}

	return 0;
}

