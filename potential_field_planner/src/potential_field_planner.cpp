#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <Eigen/Dense>
#include <math.h>

class PFPlanner{
public:
    //constructor
    PFPlanner(ros::NodeHandle *nh) {
        v_cur = Eigen::VectorXd::Zero(2);
        v_tar = Eigen::VectorXd::Zero(2);
        v_speed = Eigen::VectorXd::Zero(2);
        done_msg.data = false;
        //Read husky parameters
        if ( !nh->getParam("/target/x", tar_x )) {
            ROS_INFO("Cannot read parameter /target/x, set to default value" );
            tar_x = 0 ;
        }
        if ( !nh->getParam("/target/y", tar_y )) {
            ROS_INFO("Cannot read parameter /target/y, set to default value" );
            tar_y = 0 ;
	    }
        if ( !nh->getParam("/planner/k_att", k_att)) {
            ROS_INFO("Cannot read parameter /planner/k_att, set to default value" );
            k_att = 0 ;
	    }
        if ( !nh->getParam("/husky_velocity_controller/linear/x/max_velocity", max_lin )) {
            ROS_INFO("Cannot read parameter linear/x/max_velocity, set to default value" );
            max_lin = 0.5 ;
        }
        if ( !nh->getParam("/husky_velocity_controller/angular/z/max_velocity", max_ang )) {
            ROS_INFO("Cannot read parameter angular/z/max_velocity, set to default value" );
            max_ang = 0.5 ;
        }

        v_tar(0) = tar_x;
        v_tar(1) = tar_y;

        cur_pose_sub = nh->subscribe("/husky_velocity_controller/feedback/pose",1,&PFPlanner::cur_pose_callback, this);
        start_srv = nh->advertiseService("/planner/start", &PFPlanner::start_srv_callback, this); 
        
        twist_pub = nh->advertise<geometry_msgs::Twist>("/planner/twist",1); 
        pose_pub = nh->advertise<geometry_msgs::Pose>("/planner/pose",1); 
        done_pub = nh->advertise<std_msgs::Bool>("/planner/done",1);

    }

    void cur_pose_callback(const geometry_msgs::Pose::ConstPtr& msg){
        //save current pose
        cur_pose.position.x = msg->position.x;
        cur_pose.position.y = msg->position.y;
        cur_pose.position.z = 0;

        if (start_flag == true){
            //publish pose and twist
            get_velocity();
            pose_publisher();
            twist_publisher();
        }
        arrival_check();
    }

    bool start_srv_callback(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res){
        //set flag to true so that the planner starts
        start_flag = true;

        res.success = true;
        res.message = "Planner started";

        return true;
    }


private:
    double k_att;
    double tar_x;
    double tar_y;
    double max_lin;
    double max_ang;
    geometry_msgs::Pose cur_pose;
    ros::Subscriber cur_pose_sub;
    ros::ServiceServer start_srv;
    ros::Publisher pose_pub;
    ros::Publisher twist_pub;
    ros::Publisher done_pub;
    bool start_flag = false;
    Eigen::VectorXd v_cur;
    Eigen::VectorXd v_tar;
    Eigen::VectorXd v_speed;
    geometry_msgs::Twist twist_msg;
    std_msgs::Bool done_msg;
    geometry_msgs::Pose pose_msg;

    void get_velocity(){
        v_cur(0) = cur_pose.position.x;
        v_cur(1) = cur_pose.position.y;

        v_speed = k_att*(v_tar - v_cur);

        if (v_speed.norm() > max_lin){
            v_speed = v_speed.normalized()*max_lin;
        }
    }

    void twist_publisher(){
        twist_msg.linear.x = v_speed(0);
        twist_msg.linear.y = v_speed(1);
        twist_msg.linear.z = 0;
        twist_msg.angular.x = 0;
        twist_msg.angular.y = 0;
        twist_msg.angular.z = 0;

        twist_pub.publish(twist_msg);
    }
    
    void pose_publisher(){
        pose_msg.position.x = cur_pose.position.x + v_speed(0)/500; // 500 as the loop rate is 500
        pose_msg.position.y = cur_pose.position.y + v_speed(1)/500; // we want to find where the robot will be in 2 milliseconds 
        pose_msg.position.z = 0;

        pose_pub.publish(pose_msg);
    }

    void arrival_check(){
        if ((v_tar - v_cur).norm() < 0.1){
            start_flag = false;
            
            done_msg.data = true;
        }
        done_pub.publish(done_msg);
    }
};

int main(int argc, char **argv){
    //init ros
    ros::init(argc, argv, "twist_planner");

    // specify nodehandle
    ros::NodeHandle nh;

	ros::Rate loopRate(500);

    PFPlanner pfpobj = PFPlanner(&nh);

    while(ros::ok()){
        ros::spinOnce();
        loopRate.sleep();
    }

}