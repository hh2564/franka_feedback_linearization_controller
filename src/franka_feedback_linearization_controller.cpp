#include <franka_feedback_linearization_controller/franka_feedback_linearization_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64MultiArray.h>


#include <franka/robot_state.h>

namespace franka_feedback_linearization_controller {
    bool FrankaFeedbackLinearizationController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
    //checking to see if the default parameters can be access through node handle
    //and also getting information and interfaces  
    ee_measuredpos_pub = node_handle.advertise<geometry_msgs::Vector3Stamped>("/ee_measured_pos", 1000);
    ee_desiredpos_pub = node_handle.advertise<geometry_msgs::Vector3Stamped>("/ee_desired_pos", 1000);
    ee_measuredori_pub = node_handle.advertise<geometry_msgs::Vector3Stamped>("/ee_measured_ori", 1000);
    ee_desiredori_pub = node_handle.advertise<geometry_msgs::Vector3Stamped>("/ee_desired_ori", 1000);
    ori_error_pub = node_handle.advertise<geometry_msgs::Vector3Stamped>("/euler_error", 1000);
    joint_angle_pub = node_handle.advertise<std_msgs::Float64MultiArray>("/curr_joint_angle", 1000);

    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR("FeedbackLinearizationController: Could not read parameter arm_id");
        return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
        ROS_ERROR(
            "FeedbackLinearizationController: Invalid or no joint_names parameters provided, aborting "
            "controller init!");
        return false;
    }
    double publish_rate(30.0);
    if (!node_handle.getParam("publish_rate", publish_rate)) {
        ROS_INFO_STREAM("FeedbackLinearizationController: publish_rate not found. Defaulting to "
                        << publish_rate);
    }
    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) {
        ROS_ERROR_STREAM(
            "FeedbackLinearizationController: Error getting effort joint interface from hardware");
        return false;
    }
    for (size_t i = 0; i < 7; ++i) {
        try {
        joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "FeedbackLinearizationController: Exception getting joint handles: " << ex.what());
        return false;
        }
    }
    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR_STREAM(
            "FeedbackLinearizationController: Error getting state interface from hardware");
        return false;
    }
    try {
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
            state_interface->getHandle(arm_id + "_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "FeedbackLinearizationController: Exception getting state handle from interface: "
            << ex.what());
        return false;
    }
    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) {
        ROS_ERROR_STREAM(
            "FeedbackLinearizationController: Error getting model interface from hardware");
        return false;
    }
    try {
        model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
            model_interface->getHandle(arm_id + "_model"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "FeedbackLinearizationController: Exception getting model handle from interface: "
            << ex.what());
        return false;
    }
    std::string urdf_filename = "/home/crrl/pin_ws/src/franka_feedback_linearization_controller/urdf/panda_pin.urdf"; 
    pinocchio::urdf::buildModel(urdf_filename, model);
    data = pinocchio::Data(model);


    controlled_frame = model.getFrameId("panda_fingertip"); 

    return true;

    }

    void FrankaFeedbackLinearizationController::starting(const ros::Time& /* time */) {
        franka::RobotState initial_state = state_handle_->getRobotState();
        // convert to eigen
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
        //getting the initial position and orientation of the ee 
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        Eigen::Vector3d position_init_(initial_transform.translation());
        Eigen::Quaterniond orientation_init_(Eigen::Quaterniond(initial_transform.linear()));
        //converting from quaternion to euler angle 
        auto euler = orientation_init_.toRotationMatrix().eulerAngles(0, 1, 2);

        // get target end-effector position
        ros::param::get("/x_target", x_T);
        ros::param::get("/y_target", y_T);
        ros::param::get("/z_target", z_T);
        ros::param::get("/r_target", r_T);
        ros::param::get("/p_target", p_T);
        ros::param::get("/ya_target", ya_T);
        ros::param::get("/terminal_time", terminal_time);

        // get pd gains
        ros::param::get("/kp_delta_q", kp_delta_q);
        ros::param::get("/kd_delta_dq", kd_delta_dq);
        ros::param::get("/kd_dq", kd_dq);
        
        //getting the intial time to generate command in update 
        beginTime = ros::Time::now(); 
        MessageTime = ros::Duration(terminal_time);
        endTime = beginTime + MessageTime;

        // solve for trajctory coefficients
        A << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 2.0, 0.0, 0.0, 0.0,
            1.0, terminal_time, pow(terminal_time, 2.0), pow(terminal_time, 3.0), pow(terminal_time, 4.0), pow(terminal_time, 5.0),
            0.0, 1.0, 2.0*terminal_time, 3.0*pow(terminal_time, 2.0), 4.0*pow(terminal_time, 3.0), 5.0*pow(terminal_time, 4.0),
            0.0, 0.0, 2.0, 6.0*terminal_time, 12.0*pow(terminal_time, 2.0), 20.0*pow(terminal_time, 3.0);
        
        Ainv << A.inverse(); 
        Bx <<  position_init_[0],0,0,x_T,0,0;
        xx << Ainv*Bx; 
        By <<  position_init_[1],0,0,y_T,0,0;
        xy << Ainv*By;
        Bz <<  position_init_[2],0,0,z_T,0,0;
        xz << Ainv*Bz; 
        Br <<  euler[0],0,0,r_T,0,0;
        xr << Ainv*Br; 
        Bp <<  euler[1],0,0,p_T,0,0;
        xp << Ainv*Bp; 
        Bya <<  euler[2],0,0,ya_T,0,0;
        xya << Ainv*Bya; 

        
    
    }

     void FrankaFeedbackLinearizationController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {
        // get joint angles and angular velocities
        franka::RobotState robot_state = state_handle_->getRobotState();
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());                        
        //calculating for the time variable t to use in polynomial 
        Eigen::Affine3d curr_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d position_curr_(curr_transform.translation());
        Eigen::Quaterniond orientation_curr_(Eigen::Quaterniond(curr_transform.linear()));
        // update pinocchio robot model
        pinocchio::forwardKinematics(model, data, q, dq);
        pinocchio::updateFramePlacements(model, data);
        //converting from quaternion to euler angle 
        auto euler = orientation_curr_.toRotationMatrix().eulerAngles(0, 1, 2);
        ros::Time curTime = ros::Time::now(); 
        ros::Duration passedTime = curTime - beginTime;
        double t = passedTime.toSec(); 
         //calculating time vector t_pos, t_vel, t_acc, that could use to calculate the position, velocity, and acceleration of the ee at current time t
        // by multiplying time vector and the constant vector x<var> that we found previously 

        tpos << 1,t,pow(t,2),pow(t,3), pow(t,4),pow(t,5); 

        tvel << 0,1,2*t, 3*pow(t,2), 4*pow(t,3), 5*pow(t,4); 
        tacc << 0,0,2,6*t, 12*pow(t,2),20*pow(t,3);
        //calculating the position, velocity, acceleration in direction <var> for the ee
        double x = tpos*xx;
        double dx = tvel*xx; 
        double ddx = tacc*xx; 
        double y = tpos*xy;
        double dy = tvel*xy; 
        double ddy = tacc*xy; 
        double z = tpos*xz;
        double dz = tvel*xz; 
        double ddz = tacc*xz;
        double r = tpos*xr;
        double dr = tvel*xr; 
        double ddr = tacc*xr;
        double p = tpos*xp;
        double dp = tvel*xp;
        double ddp = tacc*xp;
        double ya = tpos*xya;
        double dya = tvel*xya; 
        double ddya = tacc*xya;

        //concentrating postion, velocity, acceleration varables into vectors respectively 
         
        X << x,y,z,r,p,ya;
        dX <<dx,dy,dz,dr,dp,dya; 
        ddX <<ddx,ddy,ddz,ddr,ddp,ddya;

        // get state variables
        std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
        Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

        // get Jacobian matrix
        pinocchio::computeFrameJacobian(model, data, q, controlled_frame, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);

        if (t > terminal_time) {
            X << x_T, y_T, z_T,r_T,p_T,ya_T;
            dX << 0.0, 0.0, 0.0,0.0, 0.0, 0.0;
            ddX << 0.0, 0.0, 0.0,0.0, 0.0, 0.0;
        }


        // get time derivative of positional Jacobian
        dJ = pinocchio::computeJointJacobiansTimeVariation(model, data, q, dq);
        error.head(3) << X[0]-position_curr_[0],X[1]-position_curr_[1],X[2]-position_curr_[2];
        double cy = cos(ya * 0.5);
        double sy = sin(ya * 0.5);
        double cp = cos(p * 0.5);
        double sp = sin(p * 0.5);
        double cr = cos(r * 0.5);
        double sr = sin(r * 0.5);

        Quaternion.w() = cr * cp * cy + sr * sp * sy;
        Quaternion.x() = sr * cp * cy - cr * sp * sy;
        Quaternion.y() = cr * sp * cy + sr * cp * sy;
        Quaternion.z() = cr * cp * sy - sr * sp * cy;
        if (Quaternion.coeffs().dot(orientation_curr_.coeffs()) < 0.0) {
            orientation_curr_.coeffs() << -orientation_curr_.coeffs();
        }
        Eigen::Quaterniond error_quaternion(orientation_curr_.inverse() * Quaternion);
        auto erroreuler = error_quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
        // error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        // error.tail(3) << curr_transform.linear() * error.tail(3);
        error.tail(3) << erroreuler[0],erroreuler[1],erroreuler[2];




        // get errors
        delta_q = jacobian.colPivHouseholderQr().solve(error);
        delta_dq = jacobian.colPivHouseholderQr().solve(dX) - dq;

        // get desired joint acceleration
        ddq_desired = jacobian.colPivHouseholderQr().solve(ddX - dJ * dq);

        // get mass matrix
        std::array<double, 49> mass_array = model_handle_->getMass();
        Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass_array.data()); 

        // compute ID controller
        torques = M * (ddq_desired + kp_delta_q * delta_q + kd_delta_dq * delta_dq) + coriolis - kd_dq * dq; 

        // set torque
        for (size_t i = 0; i < 7; ++i) {
            joint_handles_[i].setCommand(torques[i]);
        }

        ee_desired_pos.vector.x = X[0];
        ee_desired_pos.vector.y = X[1];
        ee_desired_pos.vector.z = X[2];
        ee_desired_pos.header.stamp = ros::Time::now();

        ee_measured_pos.vector.x = position_curr_[0];
        ee_measured_pos.vector.y = position_curr_[1];
        ee_measured_pos.vector.z = position_curr_[2];
        ee_measured_pos.header.stamp = ros::Time::now();

        ee_desired_ori.vector.x = X[3];
        ee_desired_ori.vector.y = X[4];
        ee_desired_ori.vector.z = X[5];
        ee_desired_ori.header.stamp = ros::Time::now();

        ee_measured_ori.vector.x = euler[0];
        ee_measured_ori.vector.y = euler[1];
        ee_measured_ori.vector.z = euler[2];
        ee_measured_ori.header.stamp = ros::Time::now();

        ee_ori_error.vector.x = error[3];
        ee_ori_error.vector.y = error[4];
        ee_ori_error.vector.z = error[5];
        ee_ori_error.header.stamp = ros::Time::now();

        std_msgs::Float64MultiArray array_msg;
        array_msg.data.resize(7);
        
        array_msg.data[0] = q[0];
        array_msg.data[1] = q[1];
        array_msg.data[2] = q[2];
        array_msg.data[3] = q[3];
        array_msg.data[4] = q[4];
        array_msg.data[5] = q[5];
        array_msg.data[6] = q[6];


        joint_angle_pub.publish(array_msg);
        ee_desiredpos_pub.publish(ee_desired_pos);
        ee_measuredpos_pub.publish(ee_measured_pos);
        ee_measuredori_pub.publish(ee_measured_ori);
        ee_desiredori_pub.publish(ee_desired_ori); 
        ori_error_pub.publish(ee_ori_error);
    }

}

PLUGINLIB_EXPORT_CLASS(franka_feedback_linearization_controller::FrankaFeedbackLinearizationController,
                       controller_interface::ControllerBase)