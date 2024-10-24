#include <pose_command_action_server/kinematic_controller.hpp>

KinematicController::KinematicController(ros::NodeHandle& nh): node_handle_(nh), loop_rate_(500){
	dim.label = "next_joint_config";
	dim.size = JOINT_ID;
	dim.stride = 1;
	layout.dim.push_back(dim);
	next_joint_config_msg.layout = layout;
	next_joint_config_msg.data.resize(7);
    counter = 0;
	//initialize variables
	fbk_joint_pos = Eigen::VectorXd::Zero(7);
	fbk_joint_vel = Eigen::VectorXd::Zero(7);
    fbk_EE_pos = Eigen::VectorXd::Zero(3);
	fbk_EE_Rmatrix = Eigen::Matrix3d::Zero(3,3);
	fbk_EE_euler = Eigen::VectorXd::Zero(3);
	motion_vec_world = Eigen::VectorXd::Zero(6);
	jacobian_local = Eigen::MatrixXd::Zero(6,7);
	jacobian_pinv = Eigen::MatrixXd::Zero(7,6);
	nullspace_projector = Eigen::MatrixXd::Zero(7,7);
	cmd_joint_vel = Eigen::VectorXd::Zero(7);
	cmd_joint_vel_nullspace = Eigen::VectorXd::Zero(7);
	cmd_joint_pos = Eigen::VectorXd::Zero(7);
	nullspace_target_eigen = Eigen::VectorXd::Zero(7);
    ref_EE_pos = Eigen::VectorXd::Zero(3);
    ref_EE_euler = Eigen::VectorXd::Zero(3);
    //ref_EE_Rmatrix = Eigen::Matrix3d::Zero(3,3);
    if(!readParameters()){
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }
    planner_pose_subscriber_ = node_handle_.subscribe("/gen3/reference/pose", 1, &KinematicController::poseCallback, this);
	planner_twist_subscriber_ = node_handle_.subscribe("/gen3/reference/twist", 1, &KinematicController::twistCallback, this);
	joint_config_pub_ = node_handle_.advertise<std_msgs::Float64MultiArray>("/gen3/joint_group_position_controller/command", 1);
	robot_joint_state_sub_ = node_handle_.subscribe("/gen3/joint_states", 1, &KinematicController::robotJointStateCallback, this);
}

KinematicController::~KinematicController(){

}
bool KinematicController::readParameters() {
	ROS_INFO("readParameters()") ;
	
    if ( !node_handle_.getParam("/c_cubic", c_cubic )) {
        ROS_INFO("Cannot read parameter /c_cubic, set to default value" );
    }
    if ( !node_handle_.getParam("/c_omega", c_omega )) {
        ROS_INFO("Cannot read parameter /c_comega, set to default value" );
    }
	if ( !node_handle_.getParam("/k_0", k_0 )) {
		ROS_INFO("Cannot read parameter /k_0" );
		return false ;
	}
	if ( !node_handle_.getParam("/c_att", c_att )) {
		ROS_INFO("Cannot read parameter /c_att" );
		return false ;
	}
	if ( !node_handle_.getParam("/gen3/target/joint/positions", nullspace_target)) { 
		ROS_INFO("Cannot read parameter /target/joint/positions" );
		return false ;
	}
	for (int i = 0; i < 7; i++){
		nullspace_target_eigen(i) = nullspace_target[i];
	}
	if ( !node_handle_.getParam("/gen3/urdf_file_name", urdf_file_name)) {
		ROS_INFO("Cannot read parameter /gen3/urdf_file_name" );
		return false ;
	}
    //build urdf model
    pinocchio::urdf::buildModel(urdf_file_name, model, false);
    pinocchio::Data data(model);
    data_ = data;
    return true;
}
void KinematicController::robotJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    //ROS_INFO("robotJointStateCallback");
    for(i = 0; i < 7; i++){
        fbk_joint_pos(i) = msg->position[i];
        fbk_joint_vel(i) = msg->velocity[i];
    }

    pinocchio::forwardKinematics(model, data_, fbk_joint_pos, fbk_joint_vel);

    fbk_EE_pos(0) = data_.oMi[7].translation()(0);
    fbk_EE_pos(1) = data_.oMi[7].translation()(1);
    fbk_EE_pos(2) = data_.oMi[7].translation()(2);

    //fbk_EE_Rmatrix = data_.oMi[7].rotation(); //fbk rotation matrix
    //pinocchio::quaternion::assignQuaternion(fbk_EE_quat, fbk_EE_Rmatrix); //convert rotation matrix to quaternion

    //fbk_EE_euler = pinocchio::rpy::matrixToRpy(fbk_EE_Rmatrix); //convert rotation matrix to euler angles

}


void KinematicController::poseCallback(const geometry_msgs::Pose::ConstPtr& msg){
    //ROS_INFO("poseCallback");
    if (counter >= 100){
        ROS_INFO("Received pose from planner: %f, %f, %f, %f, %f, %f, %f", msg->position.x, msg->position.y, msg->position.z, msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        counter = 0 ;
    }counter++;

    //get ref EE translation
    ref_EE_pos(0) = msg->position.x;
    ref_EE_pos(1) = msg->position.y;
    ref_EE_pos(2) = msg->position.z;

    motion_vec_world.head(3) = (ref_EE_pos - fbk_EE_pos)*c_cubic;
    //get ref EE orientation
    ref_EE_Rmatrix = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z).toRotationMatrix();
    se3_ref_EE_pose.rotation() = ref_EE_Rmatrix;
    ref_EE_euler = pinocchio::rpy::matrixToRpy(ref_EE_Rmatrix); //convert rotation matrix to euler angles
    //compute next EE twist SE3 transformation matrix
    se3_motion_EE = data_.oMi[7].inverse()*se3_ref_EE_pose;
    //convert SE3 transformation matrix to twist vector
    motion_vec_local = pinocchio::log6(se3_motion_EE).toVector();
    //convert twist vector to world frame
    motion_vec_world.tail(3) = c_omega*data_.oMi[7].rotation()*motion_vec_local.tail(3);
    //motion_vec_world is the next EE twist in world frame

    inverse_kinematics();
    publish_next_config();
}

void KinematicController::twistCallback(const geometry_msgs::Twist::ConstPtr& msg){
    if (counter>= 100){
        ROS_INFO("Received twist from planner: %f, %f, %f, %f, %f, %f", msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
    }/*
    //get ref EE translation vel
    motion_vec_world(0) = msg->linear.x;
    motion_vec_world(1) = msg->linear.y;
    motion_vec_world(2) = msg->linear.z;
    //get ref EE angular vel
    motion_vec_world(3) = msg->angular.x; //roll
    motion_vec_world(4) = msg->angular.y; //pitch 
    motion_vec_world(5) = msg->angular.z; //yaw
    
    inverse_kinematics();
    publish_next_config();
    */
}

void KinematicController::publish_next_config(){
    //ROS_INFO("publish_next_config");
    for (i = 0; i < 7; i++){
        next_joint_config_msg.data[i] = cmd_joint_pos(i);
    }
    joint_config_pub_.publish(next_joint_config_msg);
}

void KinematicController::inverse_kinematics(){
	//ROS_INFO("inverse_kinematics");
	//compute next EE twist SE3 transformation matrix
	pinocchio::computeAllTerms(model, data_, fbk_joint_pos, fbk_joint_vel);
	// get jacobian
	pinocchio::getJointJacobian(model, data_, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_local);
	//compute the pseudo inverse of the jacobian
	jacobian_pinv = jacobian_local.completeOrthogonalDecomposition().pseudoInverse();

	//add redundancy
	nullspace_projector = Id_mat - jacobian_pinv*jacobian_local;
	cmd_joint_vel_nullspace = k_0*(nullspace_target_eigen - fbk_joint_pos);
	
	//compute the next joint velocity
	cmd_joint_vel = jacobian_pinv*motion_vec_world + nullspace_projector*cmd_joint_vel_nullspace;

	//compute next joint pos
	cmd_joint_pos = fbk_joint_pos + c_att*cmd_joint_vel*dt;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "kinematic_controller");
    ros::NodeHandle nh;
    KinematicController kinematic_controller(nh);
    ros::Rate loop_rate(500);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}