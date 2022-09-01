#include <franka_feedback_linearization_controller/franka_feedback_linearization_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/QuaternionStamped.h>


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
    joint_velocity_pub = node_handle.advertise<std_msgs::Float64MultiArray>("/curr_joint_velocity", 1000);
    torquepub = node_handle.advertise<std_msgs::Float64MultiArray>("/tau_command", 1000);
    vpub = node_handle.advertise<std_msgs::Float64MultiArray>("/numerical_v", 1000);
    desired_qt_pub = node_handle.advertise<geometry_msgs::QuaternionStamped>("/desired_qt", 1000);

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

        Eigen::Map<Eigen::Matrix<double, 7, 1>> q(initial_state.q.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(initial_state.dq.data());
        pinocchio::forwardKinematics(model, data, q, dq);
        pinocchio::updateFramePlacements(model, data);
        beginTime = ros::Time::now(); 

        // get target end-effector position
        ros::param::get("/x_target", x_T);
        ros::param::get("/y_target", y_T);
        ros::param::get("/z_target", z_T);
        ros::param::get("/r_target", r_T);
        ros::param::get("/p_target", p_T);
        ros::param::get("/ya_target", ya_T);
        ros::param::get("/terminal_time", terminal_time);
        ros::param::get("/terminal_time2", terminal_time2);
        ros::param::get("/x_target2", x_T2);
        ros::param::get("/y_target2", y_T2);
        ros::param::get("/z_target2", z_T2);
        ros::param::get("/terminal_time3", terminal_time3);
        ros::param::get("/x_target3", x_T3);
        ros::param::get("/y_target3", y_T3);
        ros::param::get("/z_target3", z_T3);

        // get pd gains
        ros::param::get("/kp_delta_q", kp_delta_q);
        ros::param::get("/kd_delta_dq", kd_delta_dq);
        ros::param::get("/kd_dq", kd_dq);
        

        // solve for trajctory coefficients
        A << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 2.0, 0.0, 0.0, 0.0,
            1.0, terminal_time, pow(terminal_time, 2.0), pow(terminal_time, 3.0), pow(terminal_time, 4.0), pow(terminal_time, 5.0),
            0.0, 1.0, 2.0*terminal_time, 3.0*pow(terminal_time, 2.0), 4.0*pow(terminal_time, 3.0), 5.0*pow(terminal_time, 4.0),
            0.0, 0.0, 2.0, 6.0*terminal_time, 12.0*pow(terminal_time, 2.0), 20.0*pow(terminal_time, 3.0);
        
        Ainv << A.inverse(); 
        Bx <<  data.oMf[controlled_frame].translation()(0),0,0,x_T,0,0;
        xx << Ainv*Bx; 
        By <<  data.oMf[controlled_frame].translation()(1),0,0,y_T,0,0;
        xy << Ainv*By;
        Bz <<  data.oMf[controlled_frame].translation()(2),0,0,z_T,0,0;
        xz << Ainv*Bz; 
        Br <<  euler[0],0,0,euler[0],0,0;
        xr << Ainv*Br; 
        Bp <<  euler[1],0,0,euler[1],0,0;
        xp << Ainv*Bp; 
        Bya <<  euler[2],0,0,euler[2],0,0;
        xya << Ainv*Bya; 
        secstart = false;
        thirdstart = false; 

        dt1a=1+rand()%5; 
        dt1b=1+rand()%5;
        dt1c=1+rand()%5;
        dt1d=1+rand()%5; 
        dt2a=1+rand()%5; 
        dt2b=1+rand()%5;
        dt2c=1+rand()%5;
        dt2d=1+rand()%5;
        dt3a=1+rand()%5; 
        dt3b=1+rand()%5;
        dt3c=1+rand()%5;
        dt3d=1+rand()%5;
        dt4a=1+rand()%5; 
        dt4b=1+rand()%5;
        dt4c=1+rand()%5;
        dt4d=1+rand()%5;
        dt5a=1+rand()%5; 
        dt5b=1+rand()%5;
        dt5c=1+rand()%5;
        dt5d=1+rand()%5;
        dt7a=1+rand()%5; 
        dt7b=1+rand()%5;
        dt7c=1+rand()%5;
        dt7d=1+rand()%5;
        dt6a=1+rand()%5; 
        dt6b=1+rand()%5;
        dt6c=1+rand()%5;
        dt6d=1+rand()%5;

    
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

        if (t > terminal_time && secstart == false && t < terminal_time+terminal_time2 ) {
            A2 << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 2.0, 0.0, 0.0, 0.0,
                1.0, terminal_time2, pow(terminal_time2, 2.0), pow(terminal_time2, 3.0), pow(terminal_time2, 4.0), pow(terminal_time2, 5.0),
                0.0, 1.0, 2.0*terminal_time2, 3.0*pow(terminal_time2, 2.0), 4.0*pow(terminal_time2, 3.0), 5.0*pow(terminal_time2, 4.0),
                0.0, 0.0, 2.0, 6.0*terminal_time2, 12.0*pow(terminal_time2, 2.0), 20.0*pow(terminal_time2, 3.0);
            Ainv2 << A2.inverse(); 
            Bx <<  data.oMf[controlled_frame].translation()(0),0,0,x_T2,0,0;
            xx << Ainv2*Bx; 
            By <<  data.oMf[controlled_frame].translation()(1),0,0,y_T2,0,0;
            xy << Ainv2*By;
            Bz <<  data.oMf[controlled_frame].translation()(2),0,0,z_T2,0,0;
            xz << Ainv2*Bz; 
            secstart = true; 
        }

        if (t > terminal_time+terminal_time2 && thirdstart == false) {
            A3 << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 2.0, 0.0, 0.0, 0.0,
                1.0, terminal_time3, pow(terminal_time3, 2.0), pow(terminal_time3, 3.0), pow(terminal_time3, 4.0), pow(terminal_time3, 5.0),
                0.0, 1.0, 2.0*terminal_time3, 3.0*pow(terminal_time3, 2.0), 4.0*pow(terminal_time3, 3.0), 5.0*pow(terminal_time3, 4.0),
                0.0, 0.0, 2.0, 6.0*terminal_time3, 12.0*pow(terminal_time3, 2.0), 20.0*pow(terminal_time3, 3.0);
            Ainv3 << A3.inverse(); 
            Bx <<  data.oMf[controlled_frame].translation()(0),0,0,x_T3,0,0;
            xx << Ainv3*Bx; 
            By <<  data.oMf[controlled_frame].translation()(1),0,0,y_T3,0,0;
            xy << Ainv3*By;
            Bz <<  data.oMf[controlled_frame].translation()(2),0,0,z_T3,0,0;
            xz << Ainv3*Bz; 
            thirdstart = true; 
        }

        if (t > terminal_time && terminal_time+terminal_time2 > t) {
            double t2 = t-terminal_time; 
            tpos << 1,t2,pow(t2,2),pow(t2,3), pow(t2,4),pow(t2,5); 
            tvel << 0,1,2*t2, 3*pow(t2,2), 4*pow(t2,3), 5*pow(t2,4); 
            tacc << 0,0,2,6*t2, 12*pow(t2,2),20*pow(t2,3);
        }

        if (terminal_time+terminal_time2 < t) {
            double t3 = t-terminal_time-terminal_time2; 
            tpos << 1,t3,pow(t3,2),pow(t3,3), pow(t3,4),pow(t3,5); 
            tvel << 0,1,2*t3, 3*pow(t3,2), 4*pow(t3,3), 5*pow(t3,4); 
            tacc << 0,0,2,6*t3, 12*pow(t3,2),20*pow(t3,3);
        }

        
        
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
         
        X << x,y,z,euler[0],euler[1],euler[2];
        dX <<dx,dy,dz,0,0,0; 
        ddX <<ddx,ddy,ddz,0,0,0;

        // get state variables
        std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
        Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

        // get Jacobian matrix
        pinocchio::computeFrameJacobian(model, data, q, controlled_frame, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);

        if (t > terminal_time+terminal_time2 + terminal_time3) {
            X << x_T3, y_T3, z_T3,euler[0],euler[1],euler[2];
            dX << 0.0, 0.0, 0.0,0.0, 0.0, 0.0;
            ddX << 0.0, 0.0, 0.0,0.0, 0.0, 0.0;
        }


        // get time derivative of positional Jacobian
        dJ = pinocchio::computeJointJacobiansTimeVariation(model, data, q, dq);
        error.head(3) << X[0]-data.oMf[controlled_frame].translation()(0),X[1]-data.oMf[controlled_frame].translation()(1),X[2]-data.oMf[controlled_frame].translation()(2);
        Quaternion = Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())* Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY())* Eigen::AngleAxisd(ya, Eigen::Vector3d::UnitZ());
        if (Quaternion.coeffs().dot(orientation_curr_.coeffs()) < 0.0) {
            orientation_curr_.coeffs() << -orientation_curr_.coeffs();
        }
        error.tail(3) << 0,0,0; 
        // Eigen::Quaterniond error_quaternion(orientation_curr_.inverse() * Quaternion);
        // error_quaternion.normalize(); 
        // error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        // error.tail(3) << curr_transform.linear() * error.tail(3);

        // eta_d = Quaternion.w(); 
        // eta = orientation_curr_.w(); 
        // quat_d << Quaternion.x(),Quaternion.y(),Quaternion.z(); 
        // quat << orientation_curr_.x(),orientation_curr_.y(),orientation_curr_.z(); 
        // e_o = eta_d*quat -eta*quat_d-quat_d.cross(quat); 
        // derror << error[0], error[1], error[2], e_o[0], e_o[1], e_o[2]; 



        // get errors
        delta_q = jacobian.colPivHouseholderQr().solve(error);
        delta_dq = jacobian.colPivHouseholderQr().solve(dX) - dq;
        // delta_q = jacobian.colPivHouseholderQr().solve(derror);

        // get desired joint acceleration
        ddq_desired = jacobian.colPivHouseholderQr().solve(ddX - dJ * dq);

        // get mass matrix
        std::array<double, 49> mass_array = model_handle_->getMass();
        Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass_array.data()); 

        // compute ID controller
        double dt1 = cos(dt1a-dt1b*t)+sin(1/dt1c*t)-cos(dt1d+t);
        double dt2 = cos(dt2a-dt2b*t)+sin(1/dt2c*t)-cos(dt2d+t);
        double dt3 = cos(dt3a-dt3b*t)+sin(1/dt3c*t)-cos(dt3d+t);
        double dt4 = cos(dt4a-dt4b*t)+sin(1/dt4c*t)-cos(dt4d+t);
        double dt5 = cos(dt5a-dt5b*t)+sin(1/dt5c*t)-cos(dt5d+t);
        double dt6 = cos(dt6a-dt6b*t)+sin(1/dt6c*t)-cos(dt6d+t);
        double dt7 = cos(dt7a-dt7b*t)+sin(1/dt7c*t)-cos(dt7d+t);

        vt << dt1,dt2,dt3,dt4,dt5,dt6,dt7; 
        vt = 0.01*vt;
        torques = M * (ddq_desired + kp_delta_q * delta_q + kd_delta_dq * delta_dq) + coriolis + vt; 
        // torques = M * (ddq_desired + kp_delta_q * delta_q ) + coriolis - kd_dq * dq; 

        for (size_t i = 0; i < 7; ++i) {
            joint_handles_[i].setCommand(torques[i]);
        }  

        std_msgs::Float64MultiArray tau; 
        for (size_t i = 0; i < 7; ++i) {
            tau.data.push_back(torques[i]);
        }

        des_qt.quaternion.x = Quaternion.x(); 
        des_qt.quaternion.y = Quaternion.y(); 
        des_qt.quaternion.z = Quaternion.z(); 
        des_qt.quaternion.w = Quaternion.w(); 
        des_qt.header.stamp = ros::Time::now();
        ee_desired_pos.vector.x = X[0];
        ee_desired_pos.vector.y = X[1];
        ee_desired_pos.vector.z = X[2];
        ee_desired_pos.header.stamp = ros::Time::now();

        ee_measured_pos.vector.x = data.oMf[controlled_frame].translation()(0);
        ee_measured_pos.vector.y = data.oMf[controlled_frame].translation()(1);
        ee_measured_pos.vector.z = data.oMf[controlled_frame].translation()(2);
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

        std_msgs::Float64MultiArray velocity_msg;
        velocity_msg.data.resize(7);
        
        velocity_msg.data[0] = dq[0];
        velocity_msg.data[1] = dq[1];
        velocity_msg.data[2] = dq[2];
        velocity_msg.data[3] = dq[3];
        velocity_msg.data[4] = dq[4];
        velocity_msg.data[5] = dq[5];
        velocity_msg.data[6] = dq[6];

        std_msgs::Float64MultiArray v_msg;
        v_msg.data.resize(7);
        
        v_msg.data[0] = vt[0];
        v_msg.data[1] = vt[1];
        v_msg.data[2] = vt[2];
        v_msg.data[3] = vt[3];
        v_msg.data[4] = vt[4];
        v_msg.data[5] = vt[5];
        v_msg.data[6] = vt[6];


        joint_velocity_pub.publish(velocity_msg);
        joint_angle_pub.publish(array_msg);
        ee_desiredpos_pub.publish(ee_desired_pos);
        ee_measuredpos_pub.publish(ee_measured_pos);
        ee_measuredori_pub.publish(ee_measured_ori);
        ee_desiredori_pub.publish(ee_desired_ori); 
        ori_error_pub.publish(ee_ori_error);
        desired_qt_pub.publish(des_qt); 
        torquepub.publish(tau);
        vpub.publish(v_msg); 
    }

}

PLUGINLIB_EXPORT_CLASS(franka_feedback_linearization_controller::FrankaFeedbackLinearizationController,
                       controller_interface::ControllerBase)