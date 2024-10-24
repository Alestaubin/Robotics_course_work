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
        initialized = false;
        dt = 1/publish_rate;
        j = 0;

        jacobian_local = Eigen::MatrixXd::Zero(6,7);
        jacobian_pinv = Eigen::MatrixXd::Zero(7,6);
        jacobian_3_pinv = Eigen::MatrixXd::Zero(7,3);
        jacobian_local_dot = Eigen::MatrixXd::Zero(6,7);
        jacobian_3_local_dot = Eigen::MatrixXd::Zero(3,7);

        ref_EE_pos = Eigen::VectorXd::Zero(3);
        ref_EE_vel = Eigen::VectorXd::Zero(3);
        ref_EE_eff = Eigen::VectorXd::Zero(3);
        ref_EE_acc = Eigen::VectorXd::Zero(3);

        ref_joint_pos = Eigen::VectorXd::Zero(7);
        ref_joint_vel = Eigen::VectorXd::Zero(7);
        ref_joint_acc = Eigen::VectorXd::Zero(7);

        fbk_joint_pos = Eigen::VectorXd::Zero(7);
        fbk_joint_vel = Eigen::VectorXd::Zero(7);
        fbk_EE_pos = Eigen::VectorXd::Zero(3);
        fbk_EE_vel = Eigen::VectorXd::Zero(3);

        cmd_EE_acc = Eigen::VectorXd::Zero(3); 
        cmd_joint_acc = Eigen::VectorXd::Zero(7);
        cmd_joint_eff = Eigen::VectorXd::Zero(7);

        target_EE_pos = Eigen::VectorXd::Zero(3);
        start_EE_pos = Eigen::VectorXd::Zero(3);

        P_redundancy_matrix = Eigen::MatrixXd::Zero(7,7);
        Id_mat = Eigen::MatrixXd::Identity(7,7);
        tau_0 = Eigen::VectorXd::Zero(7);
        nullspace_target_eigen = Eigen::VectorXd::Zero(7);

        //set message data
        dim.label = "joint_effort";
        dim.size = 7;
        dim.stride = 1;
        layout.dim.push_back(dim);
        joint_eff_msg.layout = layout;
        joint_eff_msg.data.resize(7);
        joint_eff_msg.data.resize(7);

        //read urdf file name
        if (!nh->getParam("/gen3/urdf_file_name", urdf_file_name)){
            ROS_INFO("Cannot read parameter /gen3/urdf_file_name, set to default value" );
            urdf_file_name = "gen3_description/urdf/GEN3_URDF_V12.urdf";
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
        //Read K_att_0 param
        if ( !nh->getParam("/k_att_0", k_att_0 )) {
            ROS_INFO("Cannot read parameter k_att_0, set to default value" );
            k_att_0 = 1;
        }
        //Read D_mat_0 param
        if ( !nh->getParam("/damping_0", D_mat_0 )) {
            ROS_INFO("Cannot read parameter Damping_0, set to default value" );
            D_mat_0 = 5;
        }
        //Read K_mat_0 param
        if ( !nh->getParam("/stiffness_0", K_mat_0 )) {
            ROS_INFO("Cannot read parameter Stiffness_0, set to default value" );
            K_mat_0 = 5;
        }
        //read target hand position
        if ( !nh->getParam("/gen3/target/hand/position", init_hand_pos)) {
            ROS_INFO("Cannot read parameter /target/hand/positions, set to default value" );
            init_hand_pos = {1,1,1};
        }

        //read null-space target joint configuration
        if ( !nh->getParam("/gen3/target/joint/positions", nullspace_target)) { 
            ROS_INFO("Cannot read parameter /target/joint/positions, set to default value" );
            nullspace_target = {0,0,0,0,0,0,0};
        }
        for (int i = 0; i < 7; i++){
            nullspace_target_eigen(i) = nullspace_target[i];
            ROS_INFO("nullspace_target_eigen(%d) = %f", i, nullspace_target_eigen(i));
        }
        //load initial position of hand into end variables
        target_EE_pos(0) = init_hand_pos[0];
        target_EE_pos(1) = init_hand_pos[1];
        target_EE_pos(2) = init_hand_pos[2];

        move_to = nh->advertiseService("/pose_planner/move_to", &TSPlanner::service_call_back, this);

        cur_joint_sub = nh->subscribe("/gen3/joint_states",1,&TSPlanner::cur_joint_callback, this);

        joint_effort_pub = nh->advertise<std_msgs::Float64MultiArray>("/gen3/joint_group_effort_controller/command",1);
    }

private:
    //--Fields--//

    //pinocchio stuff
    pinocchio::Model model; 
    pinocchio::Data data_;
    const int JOINT_ID = 7;
    double dt;
    std::string urdf_file_name;
    int j;

    //publisher and subscriber stuff
    std_msgs::Float64MultiArray joint_eff_msg;
    std_msgs::MultiArrayDimension dim;
    std_msgs::MultiArrayLayout layout;
    ros::ServiceServer move_to;
    ros::Subscriber cur_joint_sub;
    ros::Publisher joint_effort_pub;

    // Limits
    Eigen::VectorXd joint_vel_max;
    Eigen::VectorXd joint_effort_max;
    Eigen::VectorXd joint_pos_max;
    Eigen::VectorXd joint_pos_min;

    //planner stuff
    std::vector<double> init_hand_pos;
    Eigen::VectorXd start_EE_pos;
    Eigen::VectorXd target_EE_pos;

    //planner output
    Eigen::VectorXd ref_EE_pos;
    Eigen::VectorXd ref_EE_vel;
    Eigen::VectorXd ref_EE_eff;
    Eigen::VectorXd ref_EE_acc;
    Eigen::VectorXd ref_joint_pos;
    Eigen::VectorXd ref_joint_vel;
    Eigen::VectorXd ref_joint_acc;

    // FeedBack
    Eigen::VectorXd fbk_joint_vel;
    Eigen::VectorXd fbk_joint_pos;
    Eigen::VectorXd fbk_joint_eff;
    Eigen::VectorXd fbk_EE_pos;
    Eigen::VectorXd fbk_EE_vel;

    //commands
    Eigen::VectorXd cmd_joint_acc;
    Eigen::VectorXd cmd_EE_acc;
    double D_mat;
    double K_mat;
    double k_att;
    Eigen::VectorXd cmd_joint_eff;

    Eigen::MatrixXd jacobian_local;
    Eigen::MatrixXd jacobian_3_local;
    Eigen::MatrixXd jacobian_local_dot;
    Eigen::MatrixXd jacobian_3_local_dot;
    Eigen::MatrixXd jacobian_pinv;
    Eigen::MatrixXd jacobian_3_pinv;

    //redundancy resolution stuff
    Eigen::MatrixXd P_redundancy_matrix;
    Eigen::MatrixXd Id_mat;
    Eigen::VectorXd tau_0;
    std::vector<double> nullspace_target;
    Eigen::VectorXd nullspace_target_eigen;
    double k_att_0;
    double D_mat_0;
    double K_mat_0;
    Eigen::VectorXd ref_EE_vel_0;

    double elapsed_time;
    bool first_call;
    bool start_flag;
    double start_time, curr_time, total_time;
    int i;
    bool initialized;

    //--callback stuff--//
    void cur_joint_callback(const sensor_msgs::JointState::ConstPtr& msg){
        //save current joint positions
        for (i = 0; i < 7; i++){
            fbk_joint_pos(i) = msg->position[i];
            fbk_joint_vel(i) = msg->velocity[i];
        }
        pinocchio::computeAllTerms(model, data_, fbk_joint_pos, fbk_joint_vel);
        
        pinocchio::forwardKinematics(model, data_, fbk_joint_pos, fbk_joint_vel);

        //save fbk EE position
        fbk_EE_pos(0) = data_.oMi[7].translation()(0);
        fbk_EE_pos(1) = data_.oMi[7].translation()(1);
        fbk_EE_pos(2) = data_.oMi[7].translation()(2);

        //get jacobian
        pinocchio::getJointJacobian(model, data_, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_local);
        //ROS_INFO("jacobian_local: \n %s", toString(jacobian_local).c_str());
        //get fbk EE vel 
        fbk_EE_vel = jacobian_local.topRows(3)*fbk_joint_vel;

        if(!start_flag){
            //initialize arm to the specified start position 
            potential_planner();
            return;
        }

        curr_time = ros::Time::now().toSec();

        if (first_call){
            start_time = curr_time;
            start_EE_pos = fbk_EE_pos;
            first_call = false; 
        }

        if (curr_time - start_time < total_time){
            //compute next EE pose
            cubic_polynomial_planner();            
            return;
        }else{
            // set current pos as the target pos for the potential field
            fbk_EE_pos = target_EE_pos;
            //reset start flag so that the potential planner will get triggered
            start_flag = false;
        }
    }
    
    bool service_call_back(highlevel_msgs::MoveTo::Request &req, highlevel_msgs::MoveTo::Response &res){
        //check input for errors
        if (req.T <= 0 || req.z <= 0){
            res.confirmation = false;
            return false;
        }
        //save end pose and time
        total_time = req.T;
        target_EE_pos(0) = req.x;
        target_EE_pos(1) = req.y;
        target_EE_pos(2) = req.z;
        //set start flag
        start_flag = true;
        first_call = true;
        res.confirmation = true;
        return true;
    }

    //--Planner stuff--//
    void potential_planner(){
        //perform calculations
        //potential field to get ref velocity
        ref_EE_vel = k_att*(target_EE_pos - fbk_EE_pos);

        // obtain ref acc and pos from the vel
        ref_EE_acc = (ref_EE_vel - fbk_EE_vel)/dt;
        ref_EE_pos = fbk_EE_pos + ref_EE_vel*dt;

        // calculate cmd_EE_acc
        cmd_EE_acc = ref_EE_acc + D_mat*(ref_EE_vel - fbk_EE_vel) + K_mat*(ref_EE_pos - fbk_EE_pos);

        // inverse dynamics
        inverse_dynamics();
        //publish next joint effort
        publish_joint_eff();
    }

    void cubic_polynomial_planner(){
        elapsed_time = curr_time - start_time;
        if(elapsed_time > total_time){
            elapsed_time = total_time;
        }
        //calculate next EE pose
        ref_EE_pos = start_EE_pos + ((3*pow(elapsed_time, 2))/pow(total_time, 2)-(2*pow(elapsed_time,3))/pow(total_time,3))*(target_EE_pos - start_EE_pos);
        //calculate next EE velocity
        ref_EE_vel = ((6*elapsed_time)/(pow(total_time,2)) - (6*pow(elapsed_time,2))/(pow(total_time,3)))*(target_EE_pos - start_EE_pos);
        //calculate next EE acceleration
        ref_EE_acc = (6/(pow(total_time,2)) - (12*elapsed_time)/(pow(total_time,3)))*(target_EE_pos - start_EE_pos);

        // calculate cmd_EE_acc
        cmd_EE_acc = ref_EE_acc + D_mat*(ref_EE_vel - fbk_EE_vel) + K_mat*(ref_EE_pos - fbk_EE_pos);

        //compute next joint configuration
        inverse_dynamics();
        //publish next joint configuration
        publish_joint_eff();
    }

    //--Controller stuff--//
    void inverse_dynamics(){
        //get jacobian time variation
        pinocchio::getJointJacobianTimeVariation(model, data_, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_local_dot);
        //get 3x7 time variation jacobian
        //ROS_INFO("jacobian_local_dot: \n %s", toString(jacobian_local_dot).c_str());
        jacobian_3_local_dot = jacobian_local_dot.topRows(3);
        //ROS_INFO("jacobian_3_local_dot: \n %s", toString(jacobian_3_local_dot).c_str());
        //get 3x7 jacobian inverse
        jacobian_3_pinv = jacobian_local.topRows(3); //3x7 matrix
        //ROS_INFO("jacobian_3_pinv: \n %s", toString(jacobian_3_pinv).c_str());
        //compute the pseudo inverse of the jacobian
        jacobian_3_pinv = jacobian_3_pinv.completeOrthogonalDecomposition().pseudoInverse();
        //ROS_INFO("jacobian_3_pinv: \n %s", toString(jacobian_3_pinv).c_str());

        //compute joint velocities
        ref_joint_vel = jacobian_3_pinv*ref_EE_vel;

        //normalize joint velocities
        for (i = 0; i < 7; i++){
            if (ref_joint_vel(i) > joint_vel_max(i)){
                ref_joint_vel(i) = joint_vel_max(i);
            }else if (ref_joint_vel(i) < -joint_vel_max(i)){
                ref_joint_vel(i) = -joint_vel_max(i);
            }
        }

        //compute joint accelerations
        cmd_joint_acc = jacobian_3_pinv*cmd_EE_acc - jacobian_3_pinv*jacobian_3_local_dot*ref_joint_vel;
        
        //compute joint torques
        cmd_joint_eff = data_.M*cmd_joint_acc + data_.nle;

        add_redundancy();
    }

    //--Redundancy resolution stuff--//
    void add_redundancy(){
        jacobian_3_local = jacobian_local.topRows(3);
        //compute P_redundancy_matrix
        P_redundancy_matrix = Id_mat - jacobian_3_local.transpose()*(jacobian_3_local*data_.M.inverse()*jacobian_3_local.transpose()).inverse()*jacobian_3_local*data_.M.inverse();

        cmd_joint_acc = k_att_0*(nullspace_target_eigen - fbk_joint_pos);

        tau_0 = data_.M*cmd_joint_acc + data_.nle; 

        //compute cmd_joint_eff
        cmd_joint_eff = cmd_joint_eff + P_redundancy_matrix*tau_0;
    }

    void publish_joint_eff(){
        //set message data
        for(i = 0; i < 7; i++){
            joint_eff_msg.data[i] = cmd_joint_eff(i);
        }
        joint_effort_pub.publish(joint_eff_msg);
        counter();
    }

    static std::string toString(const Eigen::MatrixXd& mat){
        std::stringstream ss;
        ss << mat;
        return ss.str();
    }
    void counter(){
        j++;
        if (j > 250){
            for (j = 0; j < 3; j++){
                ROS_INFO("fbk_EE_pos(%d) = %f", j, fbk_EE_pos(j));
                ROS_INFO("ref_EE_pos(%d) = %f", j, ref_EE_pos(j));
            }j = 0;
            if ((fbk_EE_pos-target_EE_pos).norm() < 0.1){
                ROS_INFO("-----WHITHIN TOLERANCE-----");
            }else{
                ROS_INFO("-----OUTSIDE TOLERANCE-----");
            }
        }
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