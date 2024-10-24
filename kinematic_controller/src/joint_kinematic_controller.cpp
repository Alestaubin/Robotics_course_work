#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <Eigen/Dense>
#include <math.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>

class JSPlanner{
public:
    JSPlanner(ros::NodeHandle *nh, double publish_rate){
        //initialize values
        publish_rate = publish_rate;
        dim.label = "joint_vel";
        dim.size = 7;
        dim.stride = 1;
        layout.dim.push_back(dim);
        joint_pos_msg.layout = layout;
        joint_pos_msg.data.resize(7);

        cur_joint_pos = Eigen::VectorXd::Zero(7);
        joint_vel = Eigen::VectorXd::Zero(7);
        tar_joint_pos = Eigen::VectorXd::Zero(7);
        k_att = 1;
        done_flag = false;

        //Read Kortex target position
        if ( !nh->getParam("/gen3/target/joint/positions", tar_joint_pos_arr)) {
            ROS_INFO("Cannot read parameter /gen3/target/joint/positions, set to default value" );
            tar_joint_pos_arr = {0,0,0,0,0,0,0};
        }

        set_tar_joint_pos();

        //Read joint max speed
        if ( !nh->getParam("/gen3/joint/max_velocity", joint_max_speed )) {
            ROS_INFO("Cannot read parameter /gen3/joint/max_velocity, set to default value" );
            joint_max_speed = 0.5;
        }

        cur_joint_sub = nh->subscribe("/gen3/joint_states",1,&JSPlanner::cur_joint_callback, this);

        joint_pos_pub = nh->advertise<std_msgs::Float64MultiArray>("/gen3/joint_group_position_controller/command",1);

    }
private:

    //--Fields--//

    ros::Subscriber cur_joint_sub;
    ros::Publisher joint_pos_pub;

    bool done_flag;
    std::vector<double> tar_joint_pos_arr;
    double joint_max_speed;
    double publish_rate;
    double k_att;

    Eigen::VectorXd tar_joint_pos;
    Eigen::VectorXd cur_joint_pos;
    Eigen::VectorXd joint_vel;

    std_msgs::Float64MultiArray joint_pos_msg;
    std_msgs::MultiArrayDimension dim;
    std_msgs::MultiArrayLayout layout;

    //--Methods--//
    
    void cur_joint_callback(const sensor_msgs::JointState::ConstPtr& msg){
        //save current joint positions
        for(int i= 0 ; i < 7; i++){
            cur_joint_pos(i) = msg->position[i];
            ROS_INFO("saving cur pos");
            ROS_INFO("cur_joint_pos(%d) = %f", i, cur_joint_pos(i));
        }

        compute_joint_vel();

        publish_joint_pos();
        
        arrival_check();
    }

    void set_tar_joint_pos(){
        for (int i = 0; i < 7; i++){
            tar_joint_pos(i) = tar_joint_pos_arr[i];
            ROS_INFO("tar_joint_pos(%d) = %f", i, tar_joint_pos(i));
        }
    }

    void arrival_check(){
        if ((tar_joint_pos - cur_joint_pos).norm() < 0.1){
            done_flag = true;
        }
    }
    
    void compute_joint_vel(){
        //compute joint speed
        if (done_flag){
            //stop robot
            for (int i = 0; i < 7; i++){
                joint_vel(i) = 0;
            }return;
        }
        
        joint_vel = k_att*(tar_joint_pos - cur_joint_pos);

        for (int i = 0; i < 7; i++){
            ROS_INFO("joint_vel(%d) = %f", i, joint_vel(i));
            if (abs(joint_vel(i)) > joint_max_speed){
                if (joint_vel(i) > 0){
                    joint_vel(i) = joint_max_speed;
                }else{
                    joint_vel(i) = -joint_max_speed;
                }
            }ROS_INFO("adjusted joint_vel(%d) = %f", i, joint_vel(i));
        }
    }

    void publish_joint_pos(){
        //publish joint position to robot
        for(int i = 0; i < 7; i++){
            if (done_flag){
                joint_pos_msg.data[i] = tar_joint_pos(i);
            }else{   
                joint_pos_msg.data[i] = cur_joint_pos(i) + joint_vel(i)/50;
            }
            ROS_INFO("cur_joint_pos(%d) = %f", i, cur_joint_pos(i));
            ROS_INFO("joint_vel(%d) = %f", i, joint_vel(i));
            ROS_INFO("joint_pos_msg.data[%d] = %f", i, joint_pos_msg.data[i]);
        }
        joint_pos_pub.publish(joint_pos_msg);
    }
};



int main(int argc, char **argv){
    //init ros
    ros::init(argc, argv, "joint_controller");

    // specify nodehandle
    ros::NodeHandle nh;

    double publish_rate;
    //Read publish rate
    if ( !nh.getParam("/publish_rate", publish_rate )) {
        ROS_INFO("Cannot read parameter /publish_rate, set to default value" );
        publish_rate = 500;
    }
    ROS_INFO("publish_rate = %f", publish_rate);
	ros::Rate loopRate(publish_rate);

    JSPlanner JSPobj = JSPlanner(&nh, publish_rate);

    while(ros::ok()){
        ros::spinOnce();
        loopRate.sleep();
    }

}