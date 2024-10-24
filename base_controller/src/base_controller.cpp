#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Bool.h>

#include <std_srvs/Trigger.h>

#include <Eigen/Dense>

#include <math.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class BaseController{
public:
    BaseController(ros::NodeHandle *nh){
        if ( !nh->getParam("/target/x", tar_x )) {
            ROS_INFO("Cannot read parameter /target/x, set to default value" );
            tar_x = 0 ;
        }
        if ( !nh->getParam("/target/y", tar_y )) {
            ROS_INFO("Cannot read parameter /target/y, set to default value" );
            tar_y = 0 ;
	    }
        if ( !nh->getParam("/husky_velocity_controller/linear/x/max_velocity", max_lin )) {
            ROS_INFO("Cannot read parameter linear/x/max_velocity, set to default value" );
            max_lin = 0.5 ;
        }
        if ( !nh->getParam("/husky_velocity_controller/angular/z/max_velocity", max_ang )) {
            ROS_INFO("Cannot read parameter angular/z/max_velocity, set to default value" );
            max_ang = 0.5 ;
        }
        cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1);
        cur_pose_sub = nh->subscribe("/husky_velocity_controller/feedback/pose", 1, &BaseController::poseCallback, this);
        done_sub = nh->subscribe("/planner/done", 1, &BaseController::doneCallback, this);
        next_pose_sub = nh->subscribe("/planner/pose", 1, &BaseController::nextPoseCallback, this);
        next_twist_sub = nh->subscribe("/planner/twist", 1, &BaseController::nextTwistCallback, this);
    };
private:
    ros::Publisher cmd_vel_pub;
    ros::Subscriber cur_pose_sub;
    ros::Subscriber done_sub;
    ros::Subscriber next_pose_sub;
    ros::Subscriber next_twist_sub;

    geometry_msgs::Pose cur_pose;
    geometry_msgs::Quaternion cur_quat;
    geometry_msgs::Pose next_pose;
    geometry_msgs::Twist next_twist_world_frame;
    geometry_msgs::Twist next_twist_robot_frame;

    double curr_yaw;    
    double max_lin;
    double max_ang;
    double tar_x;
    double tar_y;
    bool done_flag = false;

    Eigen::VectorXd vector = Eigen::VectorXd::Zero(2);
    Eigen::Matrix2d jacobian;
    
    void doneCallback(const std_msgs::Bool::ConstPtr& msg){
        //set done_flag to true so that the planner stops
        if (msg->data == true){
            done_flag = true;
        }
    }
    void nextPoseCallback(const geometry_msgs::Pose::ConstPtr& msg){
        //save next pose
        next_pose.position.x = msg->position.x;
        next_pose.position.y = msg->position.y;
        next_pose.position.z = 0;

        //ROS_INFO("next pose: %f, %f", next_pose.position.x, next_pose.position.y);
    }
    void nextTwistCallback(const geometry_msgs::Twist::ConstPtr& msg){
        //save next twist
        next_twist_world_frame.linear.x = msg->linear.x;
        next_twist_world_frame.linear.y = msg->linear.y;
        next_twist_world_frame.linear.z = 0;
        next_twist_world_frame.angular.x = 0;
        next_twist_world_frame.angular.y = 0;
        next_twist_world_frame.angular.z = 0;

        //ROS_INFO("next twist: %f, %f", next_twist_world_frame.linear.x, next_twist_world_frame.linear.y);
        twistPublisher();
    }

    void poseCallback(const geometry_msgs::Pose::ConstPtr& msg){
        //save current pose
        cur_pose.position.x = msg->position.x;
        cur_pose.position.y = msg->position.y;
        cur_pose.position.z = 0;
        cur_pose.orientation.x = cur_quat.x = msg->orientation.x;
        cur_pose.orientation.y = cur_quat.y = msg->orientation.y;
        cur_pose.orientation.z = cur_quat.z = msg->orientation.z;
        cur_pose.orientation.w = cur_quat.w = msg->orientation.w;

        //ROS_INFO("current pose: %f, %f", cur_pose.position.x, cur_pose.position.y);
        get_yaw();
    }
    void get_yaw(){
        double roll, pitch, yaw;
        tf2::Quaternion quat_tf;

        //convert from geometry_msgs::Quaternion to tf2::Quaternion
        tf2::fromMsg(cur_quat, quat_tf);
        
        //convert from tf2::Quaternion to tf2::Matrix3x3
        tf2::Matrix3x3 R(quat_tf);

        //get roll, pitch, yaw from tf2::Matrix3x3
        R.getRPY(roll, pitch, yaw);

        // ROS_INFO("yaw: %f", yaw);
        //save yaw
        curr_yaw = yaw;
    }
    void get_vector(){
        //ROS_INFO("get vector");
        //get vector from current pose to next pose
        //ROS_INFO("displacement vector: %f, %f", next_pose.position.x - cur_pose.position.x, next_pose.position.y - cur_pose.position.y);
        vector(0) = tar_x - cur_pose.position.x;
        vector(1) = tar_y - cur_pose.position.y;
        
        //ROS_INFO("vector: %f, %f", vector(0), vector(1));
    }
    void get_jacobian(){
        //ROS_INFO("get jacobian");
        //calculate displacement vector
        get_vector();        
        //set jacobian matrix
        jacobian << cos(curr_yaw), -sin(curr_yaw)*vector(0)-cos(curr_yaw)*vector(1),
                    sin(curr_yaw), cos(curr_yaw)*vector(0)-sin(curr_yaw)*vector(1);
        //ROS_INFO("jacobian: %f, %f, %f, %f", jacobian(0,0), jacobian(0,1), jacobian(1,0), jacobian(1,1));
    }
    void twistConverter(){
        //ROS_INFO("twist converter");
        //check if the robot has arrived at the goal
        if (done_flag){
            //ROS_INFO("done_flag");
            //stop the robot
            next_twist_robot_frame.linear.x = 0;
            next_twist_robot_frame.angular.z = 0;
        }else{
            //ROS_INFO("not done_flag");
            //convert from world frame to robot frame
            get_jacobian();
            Eigen::VectorXd twist_robot_frame(2);
            Eigen::VectorXd twist_world_frame(2);
            twist_world_frame << next_twist_world_frame.linear.x, next_twist_world_frame.linear.y;
            //ROS_INFO("jacobian.inverse(): %f, %f, %f, %f", jacobian.inverse()(0,0), jacobian.inverse()(0,1), jacobian.inverse()(1,0), jacobian.inverse()(1,1));
            twist_robot_frame = jacobian.inverse()*twist_world_frame;

            next_twist_robot_frame.linear.x = twist_robot_frame(0);
            next_twist_robot_frame.angular.z = twist_robot_frame(1);
            //ROS_INFO("twist converted: %f (linear.x), %f (angular.z)", next_twist_robot_frame.linear.x, next_twist_robot_frame.angular.z);
        }
        next_twist_robot_frame.linear.y = 0;
        next_twist_robot_frame.linear.z = 0;
        next_twist_robot_frame.angular.x = 0;
        next_twist_robot_frame.angular.y = 0;
    }
    void twistPublisher(){
        twistConverter();
        // check velocity does not exceed maximum
        if (next_twist_robot_frame.linear.x > max_lin){
            next_twist_robot_frame.linear.x = max_lin;
        }else if (next_twist_robot_frame.linear.x < -max_lin){
            next_twist_robot_frame.linear.x = -max_lin;
        }
        if (next_twist_robot_frame.angular.z > max_ang){
            next_twist_robot_frame.angular.z = max_ang;
        }else if (next_twist_robot_frame.angular.z < -max_ang){
            next_twist_robot_frame.angular.z = -max_ang;
        }
        cmd_vel_pub.publish(next_twist_robot_frame);
        //ROS_INFO("twist published: %f, %f", next_twist_robot_frame.linear.x, next_twist_robot_frame.angular.z);
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle nh;

    ros::Rate loopRate(500);
    
    BaseController base_controller = BaseController(&nh);

    while(ros::ok()){
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}