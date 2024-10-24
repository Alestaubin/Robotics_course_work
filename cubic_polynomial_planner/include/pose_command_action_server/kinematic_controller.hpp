#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/math/quaternion.hpp>
#include <pinocchio/math/rpy.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <math.h>
#include <cmath>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>



class KinematicController {

	public:
	KinematicController(ros::NodeHandle& node_handle);
    virtual ~KinematicController();

    private: 

    bool readParameters();
    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void robotJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void inverse_kinematics();
    void publish_next_config();

	//! ROS node handle.
	ros::NodeHandle& node_handle_;

	//! ROS topic subscriber and subscriber.
	ros::Subscriber planner_twist_subscriber_ ;
    ros::Subscriber robot_joint_state_sub_ ;
    ros::Publisher joint_config_pub_ ;
    ros::Subscriber planner_pose_subscriber_ ;

	ros::Rate loop_rate_ ;

    Eigen::VectorXd fbk_joint_pos;
    Eigen::VectorXd fbk_joint_vel;

    Eigen::VectorXd ref_EE_pos;
    Eigen::Matrix3d ref_EE_Rmatrix;
    Eigen::VectorXd ref_EE_euler;

    Eigen::VectorXd fbk_EE_pos;
    Eigen::MatrixXd fbk_EE_Rmatrix;
    Eigen::VectorXd fbk_EE_euler;
    Eigen::VectorXd motion_vec_world;
    Eigen::VectorXd motion_vec_local;

    Eigen::MatrixXd jacobian_local;
    Eigen::MatrixXd jacobian_pinv;
    Eigen::MatrixXd nullspace_projector;
    Eigen::VectorXd cmd_joint_vel;
    Eigen::VectorXd cmd_joint_vel_nullspace;
    Eigen::VectorXd cmd_joint_pos;
    Eigen::VectorXd nullspace_target_eigen;
    std::vector<double> nullspace_target;
    
    double k_0, c_att, c_cubic, c_omega;
    int i = 0; 
    int counter = 0;

    //URDF stuff
    pinocchio::Model model; 
    pinocchio::Data data_;
    std::string urdf_file_name;

    pinocchio::SE3 se3_ref_EE_pose;
    pinocchio::SE3 se3_motion_EE;

    //message publisher stuff
    std_msgs::Float64MultiArray next_joint_config_msg;
    std_msgs::MultiArrayDimension dim;
    std_msgs::MultiArrayLayout layout;

    //constants
    const Eigen::MatrixXd Id_mat = Eigen::MatrixXd::Identity(7,7);
    const int JOINT_ID = 7;
    const double dt = 0.002;
};


