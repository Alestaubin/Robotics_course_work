#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/rnea.hpp>

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
        dt = 1/publish_rate;
        dim.label = "joint_effort";
        dim.size = 7;
        dim.stride = 1;
        layout.dim.push_back(dim);
        joint_eff_msg.layout = layout;
        joint_eff_msg.data.resize(7);
        j = 0;

        tar_joint_pos = Eigen::VectorXd::Zero(7);

        ref_joint_vel = Eigen::VectorXd::Zero(7);
        ref_joint_pos = Eigen::VectorXd::Zero(7);
        ref_joint_acc = Eigen::VectorXd::Zero(7);

        cmd_joint_acc = Eigen::VectorXd::Zero(7);
        cmd_joint_eff = Eigen::VectorXd::Zero(7);

        fbk_joint_pos = Eigen::VectorXd::Zero(7);
        fbk_joint_vel = Eigen::VectorXd::Zero(7);

        //Read k_att param 
        if ( !nh->getParam("/k_att", k_att )) {
            ROS_INFO("Cannot read parameter k_att, set to default value" );
            k_att = 1;
        }
        //Read D_mat param
        if ( !nh->getParam("/damping", D_mat )) {
            ROS_INFO("Cannot read parameter Damping, set to default value" );
            D_mat = 5;
        }
        //Read K_mat param
        if ( !nh->getParam("/stiffness", K_mat )) {
            ROS_INFO("Cannot read parameter Stiffness, set to default value" );
            K_mat = 5;
        }
        //read urdf file name
        if (!nh->getParam("/gen3/urdf_file_name", urdf_file_name)){
            ROS_INFO("Cannot read parameter /gen3/urdf_file_name" );
        }
        //build urdf model
        pinocchio::urdf::buildModel(urdf_file_name, model, false);
        pinocchio::Data data(model);
        data_ = data;

        //set limit values
        joint_vel_max = model.velocityLimit;
        joint_effort_max = model.effortLimit;
        joint_pos_max = model.upperPositionLimit;
        joint_pos_min = model.lowerPositionLimit;

        for (int j = 0 ; j< 7; j++){
            ROS_INFO("joint_vel_max(%d) = %f", j, joint_vel_max(j));
            ROS_INFO("joint_effort_max(%d) = %f", j, joint_effort_max(j));
            ROS_INFO("joint_pos_max(%d) = %f", j, joint_pos_max(j));
            ROS_INFO("joint_pos_min(%d) = %f", j, joint_pos_min(j));
        }

        //Read Kortex target position
        if ( !nh->getParam("/gen3/target/joint/positions", tar_joint_pos_arr)) {
            ROS_INFO("Cannot read parameter /gen3/target/joint/positions, set to default value" );
            tar_joint_pos_arr = {0,0,0,0,0,0,0};
        }

        set_tar_joint_pos();

        cur_joint_sub = nh->subscribe("/gen3/joint_states",1,&JSPlanner::cur_joint_callback, this);

        joint_effort_pub = nh->advertise<std_msgs::Float64MultiArray>("/gen3/joint_group_effort_controller/command",1);

    }
private:
    //--Fields--//

    ros::Subscriber cur_joint_sub;
    ros::Publisher joint_effort_pub;
    
    std::vector<double> tar_joint_pos_arr;
    double publish_rate;
    double dt; 
    double k_att;
    Eigen::VectorXd tar_joint_pos;
    std::string urdf_file_name;


    //Limits
    Eigen::VectorXd joint_vel_max;
    Eigen::VectorXd joint_effort_max;
    Eigen::VectorXd joint_pos_max;
    Eigen::VectorXd joint_pos_min;

    //FeedBack
    Eigen::VectorXd fbk_joint_pos;
    Eigen::VectorXd fbk_joint_vel;
    
    //planner output
    Eigen::VectorXd ref_joint_pos;
    Eigen::VectorXd ref_joint_vel;
    Eigen::VectorXd ref_joint_acc;

    //publish msg
    std_msgs::Float64MultiArray joint_eff_msg;
    std_msgs::MultiArrayDimension dim;
    std_msgs::MultiArrayLayout layout;

    //commands
    Eigen::VectorXd cmd_joint_acc;
    double D_mat;
    double K_mat;
    Eigen::VectorXd cmd_joint_eff;
    int j;

    //pinocchio
    pinocchio::Model model;
    pinocchio::Data data_;
    int i;

    //--Methods--//
    
    void cur_joint_callback(const sensor_msgs::JointState::ConstPtr& msg){
        //save current joint feedbacks
        for(i= 0 ; i < 7; i++){
            fbk_joint_pos(i) = msg->position[i];
            fbk_joint_vel(i) = msg->velocity[i];
        }

        compute_joint_vel();

        compute_cmd_joint_acc();

        compute_cmd_joint_eff();

        publish_joint_eff();
    }

    void set_tar_joint_pos(){
        for (int i = 0; i < 7; i++){
            tar_joint_pos(i) = tar_joint_pos_arr[i];
            ROS_INFO("tar_joint_pos(%d) = %f", i, tar_joint_pos(i));
        }
    }

    void compute_joint_vel(){
        //compute joint speed
        ref_joint_vel = k_att*(tar_joint_pos - fbk_joint_pos);

        for (i = 0; i < 7; i++){
            if (abs(ref_joint_vel(i)) > joint_vel_max(i)){
                if (ref_joint_vel(i) > 0){
                  ref_joint_vel(i) = joint_vel_max(i);
                }else{
                  ref_joint_vel(i) = -joint_vel_max(i);
                }
            }
        }
        // compute ref_joint_pos and ref_joint_acc from ref_joint_vel
        ref_joint_pos = fbk_joint_pos + ref_joint_vel*dt;

        //acceleration is the change in velocity over the change in time.
        ref_joint_acc = (ref_joint_vel-fbk_joint_vel)/dt;
    }

    void compute_cmd_joint_acc(){
        //compute joint acceleration
        //cmd_joint_acc = ref_joint_acc + D_mat*(ref_joint_vel - fbk_joint_vel) + K_mat*(ref_joint_pos - fbk_joint_pos);
        cmd_joint_acc =  D_mat*(ref_joint_vel - fbk_joint_vel) + K_mat*(ref_joint_pos - fbk_joint_pos);

    }

    void compute_cmd_joint_eff(){
        //compute joint torques
        //torque = inertia*acceleration + coriolis + gravity
        //pinocchio::computeAllTerms(model, data_, fbk_joint_pos, fbk_joint_vel);
        //cmd_joint_eff = data_.M*cmd_joint_acc + data_.nle;
        cmd_joint_eff = pinocchio::rnea(model, data_, fbk_joint_pos, fbk_joint_vel, cmd_joint_acc);
    }
 
    void publish_joint_eff(){
        //publish joint position to robot
        for(i = 0; i < 7; i++){
            
            joint_eff_msg.data[i] = cmd_joint_eff(i);
            
        }
        joint_effort_pub.publish(joint_eff_msg);
        counter();
    }

    void counter(){
        j++;
        if (j > 250){
            for (j = 0; j < 7; j++){
                ROS_INFO("ref_joint_pos(%d) = %f", j, ref_joint_pos(j));
                ROS_INFO("fbk_joint_pos(%d) = %f", j, fbk_joint_pos(j));
            }j = 0;
            if ((fbk_joint_pos-tar_joint_pos).norm() < 0.3){
                ROS_INFO("-----WHITHIN TOLERANCE-----");
            }else{
                ROS_INFO("-----OUTSIDE TOLERANCE-----");
            }
        }
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