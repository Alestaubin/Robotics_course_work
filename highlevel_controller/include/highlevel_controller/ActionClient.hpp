#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/math/quaternion.hpp>
#include <pinocchio/math/rpy.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <actionlib/client/simple_action_client.h>
#include <highlevel_msgs/PoseCommandAction.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <math.h>
#include <cmath>

class ActionClient {

	public:
		/*!
		* Constructor.
		* @param node_handle_ the ROS node handle.
		*/
		ActionClient(ros::NodeHandle& node_handle);

		/*!
		 * Destructor.
		 */
		virtual ~ActionClient();

		/*
		 * A function handling necessary actions in every loop
		 */
		void update() ;

	protected:

		void activeCallback() ;
		void feedbackCallback(const highlevel_msgs::PoseCommandFeedbackConstPtr& feedback) ;
		void doneCallback(const actionlib::SimpleClientGoalState& state, const highlevel_msgs::PoseCommandResultConstPtr& result) ;

	private:
		/*!
		* Reads and verifies the ROS parameters.
		* @return true if successful.
		*/
		bool readParameters();

		//! ROS node handle.
		ros::NodeHandle& node_handle_;

		//! ROS topic subscriber.
		ros::Subscriber feedback_subscriber_ ;

		ros::Subscriber pose_subscriber_ ;
		ros::Subscriber robot_joint_state_sub ;
		ros::Publisher joint_config_pub ;

		highlevel_msgs::PoseCommandGoal action_goal_ ;

		actionlib::SimpleActionClient<highlevel_msgs::PoseCommandAction> pose_command_action_client_ ; //("go_to_home_configuration", true); // true -> don't need ros::spin()

		int NUM_TARGETS_;
		Eigen::MatrixXd target_translation_EIGEN ;
		Eigen::MatrixXd target_orientation_EIGEN ;

		int counter_ = 0 ;
		int j = 0; 

		Eigen::MatrixXd target_translation_, target_orientation_;
		Eigen::VectorXd target_duration_;
		std::vector<double> translation_row, orientation_row;
		
		double duration;
		double k_0, c_att;
		int i = 0; 

		//constants
		const Eigen::MatrixXd Id_mat = Eigen::MatrixXd::Identity(7,7);
		const int JOINT_ID = 7;
		const double dt = 0.002;

		bool ori_done;
		bool pos_done;
};


