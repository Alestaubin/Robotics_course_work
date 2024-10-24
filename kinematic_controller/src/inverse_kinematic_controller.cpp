#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>

#include <math.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>

#include <highlevel_msgs/MoveTo.h>


class TSPlanner{
public:
    TSPlanner(ros::NodeHandle *nh, double publish_rate){
        //initialize values
        elapsed_time = 0;
        first_call = false;
        start_flag = false;
        dt = 1/publish_rate;
        start_x = start_y = start_z = curr_x = curr_y = curr_z = end_x = end_y = end_z = 0;
        start_time = curr_time = total_time = 0;

        max_joint_limits = {10, 2.41, 10, 2.66, 10, 2.21, 10};
        max_joint_speeds = {1.3963, 1.3963, 1.3963, 1.3963, 1.2218, 1.2218, 1.2218};

        jacobian_local = Eigen::MatrixXd::Zero(6,7);
        jacobian_pinv = Eigen::MatrixXd::Zero(7,6);
        jacobian_3_pinv = Eigen::MatrixXd::Zero(7,3);

        joint_config = Eigen::VectorXd::Zero(7);
        joint_vel = Eigen::VectorXd::Zero(7);
        fbk_joint_vel = Eigen::VectorXd::Zero(7);
        next_EE_vel = Eigen::VectorXd::Zero(3);

        dim.label = "next_joint_config";
        dim.size = 7;
        dim.stride = 1;
        layout.dim.push_back(dim);
        next_joint_config_msg.layout = layout;
        next_joint_config_msg.data.resize(7);
        next_joint_config_msg.data.resize(7);

        //read urdf file name
        if (!nh->getParam("/gen3/urdf_file_name", urdf_file_name)){
            ROS_INFO("Cannot read parameter /gen3/urdf_file_name, set to default value" );
            urdf_file_name = "gen3_description/urdf/GEN3_URDF_V12.urdf";
        }

        //build urdf model
        pinocchio::urdf::buildModel(urdf_file_name, model, false);
        pinocchio::Data data(model);
        data_ = data;

        //read target hand position Matrix<double, 3,1>
        if ( !nh->getParam("/gen3/target/hand/position", init_hand_pos)) {
            ROS_INFO("Cannot read parameter /target/hand/positions, set to default value" );
            init_hand_pos = {1,1,1};
        }
        if ( !nh->getParam("/gen3/joint/max_velocity", joint_max_vel )) {
            ROS_INFO("Cannot read parameter /gen3/joint/max_velocity, set to default value" );
            joint_max_vel = 0.5;
        }

        move_to = nh->advertiseService("/pose_planner/move_to", &TSPlanner::service_call_back, this);

        cur_joint_sub = nh->subscribe("/gen3/joint_states",1,&TSPlanner::cur_joint_callback, this);

        joint_config_pub = nh->advertise<std_msgs::Float64MultiArray>("/gen3/joint_group_position_controller/command",1);

        //initialize arm to the specified start position
        initialize_arm();
    }

private:
    //--Fields--//

    //pinocchio stuff
    pinocchio::Model model; 
    pinocchio::Data data_;
    const int JOINT_ID = 7;
    double dt;
    std::string urdf_file_name;

    //publisher and subscriber stuff
    std_msgs::Float64MultiArray next_joint_config_msg;
    std_msgs::MultiArrayDimension dim;
    std_msgs::MultiArrayLayout layout;
    ros::ServiceServer move_to;
    ros::Subscriber cur_joint_sub;
    ros::Publisher joint_config_pub;

    //planner stuff
    std::vector<double> init_hand_pos;
    Eigen::VectorXd joint_config;
    Eigen::VectorXd standby_joint_config;
    Eigen::VectorXd joint_vel;
    Eigen::VectorXd fbk_joint_vel;
    Eigen::VectorXd next_EE_vel;

    Eigen::MatrixXd jacobian_local;
    Eigen::MatrixXd jacobian_pinv;
    Eigen::MatrixXd jacobian_3_pinv;

    double elapsed_time;
    bool first_call;
    bool start_flag;
    double start_time, curr_time, total_time;
    double start_x, start_y, start_z, curr_x, curr_y, curr_z, end_x, end_y, end_z;
    int i;
    double next_z;
    double next_x;
    double next_y;
    double joint_max_vel;

    std::vector<double> max_joint_limits;
    std::vector<double> max_joint_speeds;

    //--callback stuff--//
    void cur_joint_callback(const sensor_msgs::JointState::ConstPtr& msg){
        ROS_INFO("cur_joint_callback");
        //save current joint positions
        if (first_call){
            for (i = 0; i < 7; i++){
                joint_config(i) = msg->position[i];
                fbk_joint_vel(i) = msg->velocity[i];
            }
        }
        pinocchio::forwardKinematics(model, data_, joint_config, joint_vel); 

        curr_x = data_.oMi[7].translation()(0);
        curr_y = data_.oMi[7].translation()(1);
        curr_z = data_.oMi[7].translation()(2);
        ROS_INFO("start_x: %f, start_y: %f, start_z: %f", start_x, start_y, start_z);
        ROS_INFO("curr_x: %f, curr_y: %f, curr_z: %f", curr_x, curr_y, curr_z);
        ROS_INFO("end_x: %f, end_y: %f, end_z: %f", end_x, end_y, end_z);

        curr_time = ros::Time::now().toSec();

        if (first_call){
            start_time = curr_time;
            start_x = curr_x;
            start_y = curr_y;
            start_z = curr_z;
            first_call = false; 
        }

        if (start_flag && curr_time - start_time < total_time){
            //compute next EE velocity 
            get_next_EE_vel();
            //compute next joint configuration
            inverse_kinematics();
            //publish next joint configuration
            publish_next_config();
            return;
        }else if (start_flag){
            //set standby joint configuration
            standby_joint_config = joint_config;
            //reset start flag
            start_flag = false;
            return;
        }
        publish_standby_config();
    }
    
    bool service_call_back(highlevel_msgs::MoveTo::Request &req, highlevel_msgs::MoveTo::Response &res){
        //check input for errors
        if (req.T <= 0 || req.z <= 0){
            res.confirmation = false;
            return false;
        }
        //save end pose and time
        total_time = req.T;
        end_x = req.x;
        end_y = req.y;
        end_z = req.z;
        //set start flag
        start_flag = true;
        first_call = true;
        res.confirmation = true;
        return true; 
    }

    //--Planner stuff--//

    void get_next_EE_vel(){
        ROS_INFO("get_next_EE_pose");
        elapsed_time = curr_time - start_time;
        // don't know why, but these don't work:
        //next_EE_vel(0) = ((6*elapsed_time)/pow(total_time, 2)-(6*pow(elapsed_time,2))/pow(total_time,3))*(end_x - start_x);
        //next_EE_vel(1) = ((6*elapsed_time)/pow(total_time, 2)-(6*pow(elapsed_time,2))/pow(total_time,3))*(end_y - start_y);
        //next_EE_vel(2) = ((6*elapsed_time)/pow(total_time, 2)-(6*pow(elapsed_time,2))/pow(total_time,3))*(end_z - start_z);
        //find next pose
        next_x = start_x + ((3*pow(elapsed_time, 2))/pow(total_time, 2)-(2*pow(elapsed_time,3))/pow(total_time,3))*(end_x - start_x);
        next_y = start_y + ((3*pow(elapsed_time, 2))/pow(total_time, 2)-(2*pow(elapsed_time,3))/pow(total_time,3))*(end_y - start_y);
        next_z = start_z + ((3*pow(elapsed_time, 2))/pow(total_time, 2)-(2*pow(elapsed_time,3))/pow(total_time,3))*(end_z - start_z);
        //calculate vel from next pose
        next_EE_vel(0) = (next_x - curr_x)*500;
        next_EE_vel(1) = (next_y - curr_y)*500;
        next_EE_vel(2) = (next_z - curr_z)*500;

        ROS_INFO("next_EE_vel(0): %f", next_EE_vel(0));
        ROS_INFO("next_EE_vel(1): %f", next_EE_vel(1));
        ROS_INFO("next_EE_vel(2): %f", next_EE_vel(2));
    }

    void initialize_arm(){
        ROS_INFO("initialize_arm");
        first_call = true;
        start_flag = true;
        //load initial position of hand into end variables
        end_x = init_hand_pos[0];
        end_y = init_hand_pos[1];
        end_z = init_hand_pos[2];
        //set total time
        total_time = 5;
    }

    //--Controller stuff--//
    void inverse_kinematics(){
        ROS_INFO("inverse_kinematics");
        //check if joint angles are within limits
        for (i = 0; i < 7; i++){
            if (joint_config(i) >= max_joint_limits[i]-dt){
                joint_config(i) = max_joint_limits[i];
                //set joint velocity to 0
                if (joint_vel(i) > 0){
                    joint_vel(i) = 0;
                }
            }else if (joint_config(i) <= -max_joint_limits[i]+dt){
                joint_config(i) = -max_joint_limits[i];
                //set joint velocity to 0
                if (joint_vel(i) < 0){
                    joint_vel(i) = 0;
                }
            }
        }
        //compute all terms
        pinocchio::computeAllTerms(model, data_, joint_config, joint_vel);
        // get jacobian
        pinocchio::getJointJacobian(model, data_, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_local);
        jacobian_3_pinv = jacobian_local.topRows(3); //3x7 matrix
        //compute the pseudo inverse of the jacobian
        jacobian_3_pinv = jacobian_3_pinv.completeOrthogonalDecomposition().pseudoInverse();
        //compute the next joint velocity
        joint_vel = jacobian_3_pinv*next_EE_vel; //7x6 matrix multiplied by a 6d vector

        for (i = 0; i < 7; i++){
            //normalize joint velocities
            ROS_INFO("next_joint_vel(%d): %f", i, joint_vel(i));
            if (abs(joint_vel(i)) > max_joint_speeds[i]){
                if (joint_vel(i) > 0){
                    joint_vel(i) = max_joint_speeds[i];
                }else{
                    joint_vel(i) = -max_joint_speeds[i];
                }
            }
            ROS_INFO("adjusted next_joint_vel(%d): %f", i, joint_vel(i));
        }
        //compute next joint configuration from next_joint_velocity
        joint_config = joint_config + joint_vel*dt;
    }

    void publish_next_config(){
        //set message data
        for(i = 0; i < 7; i++){
            //print out next joint config
            ROS_INFO("joint_config(%d): %f", i, joint_config(i));
            next_joint_config_msg.data[i] = joint_config(i);
        }
        //publish message
        joint_config_pub.publish(next_joint_config_msg);
    }

    void publish_standby_config(){
        //set message data
        for(i = 0; i < 7; i++){
            next_joint_config_msg.data[i] = standby_joint_config(i);
        }
        //publish message
        joint_config_pub.publish(next_joint_config_msg);
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "task_controller");
    ros::NodeHandle nh;
    double publish_rate;
    //Read publish rate
    if ( !nh.getParam("/publish_rate", publish_rate )) {
        ROS_INFO("Cannot read parameter /publish_rate, set to default value" );
        publish_rate = 500;
    }

    ros::Rate loopRate(publish_rate);

    TSPlanner TSPobj = TSPlanner(&nh, publish_rate);

    while(ros::ok()){
        ros::spinOnce();
        loopRate.sleep();
    }
}