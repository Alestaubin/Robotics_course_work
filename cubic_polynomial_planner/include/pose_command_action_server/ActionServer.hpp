#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/math/quaternion.hpp>
#include <pinocchio/math/rpy.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <actionlib/server/simple_action_server.h>
#include <highlevel_msgs/PoseCommandAction.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <math.h>
#include <cmath>


class ActionServer {

	public:
	/*!
	* Constructor.
	* @param node_handle_ the ROS node handle.
	*/
	ActionServer(ros::NodeHandle& node_handle);

  	/*!
  	 * Destructor.
  	 */
	virtual ~ActionServer();

	/*
	 * A function handling necessary actions in every loop
	 */
	void update() ;

	private:
	/*!
	* Reads and verifies the ROS parameters.
	* @return true if successful.
	*/
	bool readParameters();

	void move_to_ballback(const highlevel_msgs::PoseCommandGoalConstPtr& goal) ;
	void feedbackCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void print_counter();
    void planner_EE_vel();
    void planner_EE_ori();
    void publish_topics();

	//! ROS node handle.
	ros::NodeHandle& node_handle_;

	actionlib::SimpleActionServer<highlevel_msgs::PoseCommandAction> move_to_action_server_ ;

	ros::Rate loop_rate_ ;

	//! ROS topic subscriber and subscriber.
	ros::Subscriber feedback_subscriber_ ;
    ros::Publisher twist_publisher_ ;
    ros::Publisher pose_publisher_ ;

    geometry_msgs::Pose pose_msg;
    geometry_msgs::Twist twist_msg;

    highlevel_msgs::PoseCommandGoal pose_command_goal_ ;
    highlevel_msgs::PoseCommandFeedback pose_command_feedback_ ;
    highlevel_msgs::PoseCommandResult pose_command_result_ ;

    //URDF stuff
    pinocchio::Model model; 
    pinocchio::Data data_;
    const int JOINT_ID = 7;
    double dt;
    std::string urdf_file_name;

    //Eigen stuff
    Eigen::VectorXd ref_EE_vel;
    Eigen::VectorXd ref_EE_pos;
    Eigen::Matrix3d ref_EE_Rmatrix;
    Eigen::VectorXd ref_EE_euler;

    Eigen::VectorXd fbk_EE_pos;
    Eigen::VectorXd fbk_joint_vel;
    Eigen::VectorXd fbk_joint_pos;
    Eigen::Matrix3d fbk_EE_Rmatrix;
    Eigen::VectorXd fbk_EE_euler;
    Eigen::VectorXd start_EE_pos;
    Eigen::VectorXd start_EE_quat;
    Eigen::VectorXd target_EE_pos;

    //Quaternion stuff
    Eigen::Quaterniond fbk_EE_quat;
    Eigen::Quaterniond q_ini;
    Eigen::Quaterniond q_tar;
    Eigen::Quaterniond qs;
    Eigen::MatrixXd ref_EE_ang_vel;

    //other stuff
    double interpolation_parameter;
    double interpolation_parameter_dot;
    bool first_call, start_flag;
    int i,j, counter;
    double lin_max_vel;
    double total_time, start_time, curr_time, elapsed_time;
    double c_att, k_0, c_cubic, c_omega;
    pinocchio::SE3 se3_ref_EE_pose;
    pinocchio::SE3 se3_motion_EE;
    Eigen::VectorXd motion_vec_local;
    Eigen::VectorXd motion_vec_world;
};


