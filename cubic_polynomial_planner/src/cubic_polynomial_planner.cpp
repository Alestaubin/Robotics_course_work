#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/math/quaternion.hpp>
#include <pinocchio/math/rpy.hpp>

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
        first_call = false;
        start_flag = false;
        init = false;
        dt = 1/publish_rate;
        elapsed_time = i = j = start_time = curr_time = total_time = 0;

        jacobian_local = Eigen::MatrixXd::Zero(6,7);
        jacobian_pinv = Eigen::MatrixXd::Zero(7,6);

        cmd_joint_vel = Eigen::VectorXd::Zero(7);
        cmd_joint_pos = Eigen::VectorXd::Zero(7);
        ref_EE_vel = Eigen::VectorXd::Zero(3);
        ref_EE_pos = Eigen::VectorXd::Zero(3);

        target_EE_pos = Eigen::VectorXd::Zero(3);
        start_EE_pos = Eigen::VectorXd::Zero(3);
        fbk_EE_pos = Eigen::VectorXd::Zero(3);

        nullspace_projector = Eigen::MatrixXd::Zero(7,7);
        nullspace_target_eigen = Eigen::VectorXd::Zero(7);
        cmd_joint_vel_nullspace = Eigen::VectorXd::Zero(7);

        fbk_joint_vel = Eigen::VectorXd::Zero(7);
        fbk_joint_pos = Eigen::VectorXd::Zero(7);

        fbk_EE_Rmatrix = Eigen::MatrixXd::Zero(3,3);

        ref_EE_Rmatrix = Eigen::MatrixXd::Zero(3,3);
        ref_EE_euler = Eigen::VectorXd::Zero(3);

        motion_EE_Rmatrix = Eigen::MatrixXd::Zero(3,3);
        ref_EE_ang_vel = Eigen::MatrixXd::Zero(3,3);

        joint_vel_max = Eigen::VectorXd::Zero(7);

        q_ini = Eigen::Quaterniond::Identity();
        q_tar = Eigen::Quaterniond::Identity();
        qs = Eigen::Quaterniond::Identity();

        se3_ref_EE_pose = pinocchio::SE3::Identity();
        se3_motion_EE = pinocchio::SE3::Identity();
        motion_vec_local = Eigen::VectorXd::Zero(6);
        motion_vec_world = Eigen::VectorXd::Zero(6);



        //set message layout
        dim.label = "next_joint_config";
        dim.size = 7;
        dim.stride = 1;
        layout.dim.push_back(dim);
        next_joint_config_msg.layout = layout;
        next_joint_config_msg.data.resize(7);
        next_joint_config_msg.data.resize(7);

        //get k_0 parameter:
        if ( !nh->getParam("/k_0", k_0 )) {
            ROS_INFO("Cannot read parameter /k_0, set to default value" );
            k_0 = 1;
        }
        if ( !nh->getParam("/c_att", c_att )) {
            ROS_INFO("Cannot read parameter /k_0, set to default value" );
            c_att = 10;
        }
        if ( !nh->getParam("/c_cubic", c_cubic )) {
            ROS_INFO("Cannot read parameter /c_cubic, set to default value" );
            c_cubic = 10;
        }
        if ( !nh->getParam("/c_omega", c_omega )) {
            ROS_INFO("Cannot read parameter /c_cubic, set to default value" );
            c_omega = 10;
        }
        //read null-space target joint configuration
        if ( !nh->getParam("/gen3/target/joint/positions", nullspace_target)) { 
            ROS_INFO("Cannot read parameter /target/joint/positions, set to default value" );
            nullspace_target = {0,0,0,0,0,0,0};
        }
        for (int i = 0; i < 7; i++){
            nullspace_target_eigen(i) = nullspace_target[i];
        }
        ROS_INFO("nullspace_target = %f,%f,%f,%f,%f,%f,%f", nullspace_target[0], nullspace_target[1], nullspace_target[2], nullspace_target[3], nullspace_target[4], nullspace_target[5], nullspace_target[6]);

        //read urdf file name
        if (!nh->getParam("/gen3/urdf_file_name", urdf_file_name)){
            ROS_INFO("Cannot read parameter /gen3/urdf_file_name, set to default value" );
            urdf_file_name = "gen3_description/urdf/GEN3_URDF_V12.urdf";
        }
        if ( !nh->getParam("/gen3/linear/max_velocity", lin_max_vel)) {
            ROS_INFO("Cannot read parameter /gen3/linear/max_velocity, set to default value" );
            lin_max_vel = 0.5;
        }

        //build urdf model
        pinocchio::urdf::buildModel(urdf_file_name, model, false);
        pinocchio::Data data(model);
        data_ = data;

        joint_vel_max = model.velocityLimit;

        if ( !nh->getParam("/gen3/target/hand/position", init_hand_pos)) {
            ROS_INFO("Cannot read parameter /target/hand/positions, set to default value" );
            init_hand_pos = {1,1,1};
        }
        if ( !nh->getParam("/gen3/target/hand/orientation", init_hand_ori)) {
            ROS_INFO("Cannot read parameter /target/hand/orientation, set to default value" );
            init_hand_ori = {0,0,0};
        }
        ROS_INFO("init_hand_pos = %f, %f, %f", init_hand_pos[0], init_hand_pos[1], init_hand_pos[2]);
        ROS_INFO("init_hand_pos = %f, %f, %f", init_hand_ori[0], init_hand_ori[1], init_hand_ori[2]);
        move_to = nh->advertiseService("/pose_planner/move_to", &TSPlanner::move_to_callback, this);

        move_ori = nh->advertiseService("/pose_planner/move_ori", &TSPlanner::move_ori_callback, this);

        cur_joint_sub = nh->subscribe("/gen3/joint_states",1,&TSPlanner::cur_joint_callback, this);

        joint_config_pub = nh->advertise<std_msgs::Float64MultiArray>("/gen3/joint_group_position_controller/command",1);
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
    ros::ServiceServer move_ori;
    ros::Subscriber cur_joint_sub;
    ros::Publisher joint_config_pub;

    std::vector<double> init_hand_pos;
    Eigen::VectorXd standby_joint_config;

    //planner output
    Eigen::VectorXd cmd_joint_vel;
    Eigen::VectorXd cmd_joint_pos;
    Eigen::VectorXd ref_EE_vel;
    Eigen::VectorXd ref_EE_pos;

    //fbk
    Eigen::VectorXd fbk_joint_vel;
    Eigen::VectorXd fbk_joint_pos;

    //inverse kinematics
    Eigen::MatrixXd jacobian_local;
    Eigen::MatrixXd jacobian_pinv;

    //end effector
    Eigen::VectorXd start_EE_pos;
    Eigen::VectorXd target_EE_pos;
    Eigen::VectorXd fbk_EE_pos;

    Eigen::VectorXd joint_vel_max;

    //nullspace
    Eigen::MatrixXd nullspace_projector;
    Eigen::VectorXd nullspace_target_eigen;
    std::vector<double> nullspace_target;
    Eigen::VectorXd cmd_joint_vel_nullspace;
    Eigen::MatrixXd Id_mat = Eigen::MatrixXd::Identity(7,7);

    //orientations
    Eigen::Matrix3d fbk_EE_Rmatrix;
    Eigen::VectorXd fbk_EE_euler;
    Eigen::Matrix3d ref_EE_Rmatrix;
    Eigen::VectorXd ref_EE_euler;
    Eigen::MatrixXd motion_EE_Rmatrix;
    Eigen::Quaterniond fbk_EE_quat;
    Eigen::Quaterniond q_ini;
    Eigen::Quaterniond q_tar;
    Eigen::Quaterniond qs;
    double interpolation_parameter;
    std::vector<double> init_hand_ori;

    pinocchio::SE3 se3_ref_EE_pose;
    pinocchio::SE3 se3_motion_EE;
    Eigen::VectorXd motion_vec_local;
    Eigen::VectorXd motion_vec_world;

    Eigen::MatrixXd ref_EE_ang_vel; 

    double elapsed_time;
    bool first_call;
    bool start_flag, init;
    int i,j;
    double lin_max_vel;
    double total_time, start_time, curr_time;
    double c_att, k_0, c_cubic, c_omega; 

    //--callback stuff--//
    void cur_joint_callback(const sensor_msgs::JointState::ConstPtr& msg){
        counter();
        if (!init){
            initialize_arm();
            init = true;
        }
        //save current joint positions

        for (i = 0; i < 7; i++){
            fbk_joint_pos(i) = msg->position[i];
            fbk_joint_vel(i) = msg->velocity[i];
        }

        pinocchio::forwardKinematics(model, data_, fbk_joint_pos, fbk_joint_vel);

        fbk_EE_pos(0) = data_.oMi[7].translation()(0);
        fbk_EE_pos(1) = data_.oMi[7].translation()(1);
        fbk_EE_pos(2) = data_.oMi[7].translation()(2);

        fbk_EE_Rmatrix = data_.oMi[7].rotation(); //fbk rotation matrix
        pinocchio::quaternion::assignQuaternion(fbk_EE_quat, fbk_EE_Rmatrix); //convert rotation matrix to quaternion

        fbk_EE_euler = pinocchio::rpy::matrixToRpy(fbk_EE_Rmatrix); //convert rotation matrix to euler angles

        curr_time = ros::Time::now().toSec();

        if (first_call){
            start_time = curr_time;
            start_EE_pos = fbk_EE_pos; 
            q_ini = fbk_EE_quat;
            first_call = false; 
        }

        elapsed_time = curr_time - start_time;

        if(elapsed_time > total_time){
            elapsed_time = total_time;
        }  

        interpolation_parameter = 3*pow(elapsed_time, 2)/pow(total_time, 2)-(2*pow(elapsed_time,3))/pow(total_time,3);
        
        if (start_flag){
            //compute next EE velocity 
            planner_EE_vel();
            //compute next EE orientation
            planner_EE_ori();
            //compute next joint configuration
            inverse_kinematics();
            //publish next joint configuration
            publish_next_config();
            return;
        }
    }
    
    bool move_to_callback(highlevel_msgs::MoveTo::Request &req, highlevel_msgs::MoveTo::Response &res){
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

        //set target orientation to current orientation
        //q_tar = fbk_EE_quat;

        //set start flag
        start_flag = true;
        first_call = true;
        res.confirmation = true;
        return true; 
    }

    bool move_ori_callback(highlevel_msgs::MoveTo::Request &req, highlevel_msgs::MoveTo::Response &res){
        //check input for errors
        if (req.T <= 0){
            res.confirmation = false;
            return false;
        }
        //save end pose and time
        total_time = req.T;

        //convert Euler angles to quaternion
        pinocchio::quaternion::assignQuaternion(q_tar, pinocchio::rpy::rpyToMatrix(req.x, req.y, req.z));

        //set translation target to current position
        //target_EE_pos = fbk_EE_pos;

        //set start flag
        start_flag = true;
        first_call = true;
        res.confirmation = true;
        return true; 
    }

    void initialize_arm(){
        first_call = true;
        start_flag = true;
        //load initial position of hand into end variables
        target_EE_pos(0) = init_hand_pos[0];
        target_EE_pos(1) = init_hand_pos[1];
        target_EE_pos(2) = init_hand_pos[2];

        //set target orientation of hand
        pinocchio::quaternion::assignQuaternion(q_tar, pinocchio::rpy::rpyToMatrix(init_hand_ori[0], init_hand_ori[1], init_hand_ori[2]));

        ROS_INFO("init_hand_pos = %f, %f, %f", init_hand_pos[0], init_hand_pos[1], init_hand_pos[2]);
        //set total time
        total_time = 5;
    }

    //--Planner stuff--//

    void planner_EE_vel(){
        //calculate next EE pose
        ref_EE_pos = start_EE_pos + interpolation_parameter*(target_EE_pos - start_EE_pos);
        //calculate next EE velocity
        ref_EE_vel = (ref_EE_pos - fbk_EE_pos)*c_cubic;

        //normalize velocity
        if (ref_EE_vel.norm() > lin_max_vel){
            ref_EE_vel = ref_EE_vel.normalized()*lin_max_vel;
        }

        //set next position in the transformation matrix
        se3_ref_EE_pose.translation() = ref_EE_pos;
    }
    void planner_EE_ori(){
        //calculate slerp
        qs = q_ini.slerp(interpolation_parameter, q_tar);

        ref_EE_Rmatrix = qs.toRotationMatrix(); //convert quaternion to rotation matrix  

        //set next orientation in the transformation matrix
        se3_ref_EE_pose.rotation() = ref_EE_Rmatrix;

        ref_EE_euler = pinocchio::rpy::matrixToRpy(ref_EE_Rmatrix); //convert rotation matrix to euler angles
    }

    //--Controller stuff--//
    void inverse_kinematics(){
        //compute next EE twist SE3 transformation matrix
        se3_motion_EE = data_.oMi[7].inverse()*se3_ref_EE_pose;
        //convert SE3 transformation matrix to twist vector
        motion_vec_local = pinocchio::log6(se3_motion_EE).toVector();
        //convert twist vector to world frame
        motion_vec_world.head(3) = ref_EE_vel;
        motion_vec_world.tail(3) = c_omega*data_.oMi[7].rotation()*motion_vec_local.tail(3);

        pinocchio::computeAllTerms(model, data_, fbk_joint_pos, cmd_joint_vel);
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

    void publish_next_config(){
        //set message data
        for(i = 0; i < 7; i++){
            //print out next joint config
            next_joint_config_msg.data[i] = cmd_joint_pos(i);
        }
        //publish message
        joint_config_pub.publish(next_joint_config_msg);
    }

    static std::string toString(const Eigen::MatrixXd& mat){
        std::stringstream ss;
        ss << mat;
        return ss.str();
    }
    void counter(){
        j++;
        if (j > 100){
            ROS_INFO("fbk_EE_pos = %f,%f,%f", fbk_EE_pos(0), fbk_EE_pos(1), fbk_EE_pos(2));
            ROS_INFO("fbk_EE_euler = %f,%f,%f", fbk_EE_euler(0), fbk_EE_euler(1), fbk_EE_euler(2));
            ROS_INFO("ref_EE_euler = %f,%f,%f", ref_EE_euler(0), ref_EE_euler(1), ref_EE_euler(2));
            ROS_INFO("motion_vec_world = %f,%f,%f,%f,%f,%f", motion_vec_world(0), motion_vec_world(1), motion_vec_world(2), motion_vec_world(3), motion_vec_world(4), motion_vec_world(5));
            ROS_INFO("cmd_joint_vel = %f,%f,%f,%f,%f,%f,%f", cmd_joint_vel(0), cmd_joint_vel(1), cmd_joint_vel(2), cmd_joint_vel(3), cmd_joint_vel(4), cmd_joint_vel(5), cmd_joint_vel(6));
            ROS_INFO("cmd_joint_pos = %f,%f,%f,%f,%f,%f,%f", cmd_joint_pos(0), cmd_joint_pos(1), cmd_joint_pos(2), cmd_joint_pos(3), cmd_joint_pos(4), cmd_joint_pos(5), cmd_joint_pos(6));
            j=0;
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