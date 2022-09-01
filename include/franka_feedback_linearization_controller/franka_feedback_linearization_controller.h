#pragma once

#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/model.hpp"

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <dynamic_reconfigure/server.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <realtime_tools/realtime_publisher.h>

#include <franka_example_controllers/JointTorqueComparison.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <Eigen/Dense>
#include<Eigen/Eigen>
#include <Eigen/Core>

namespace franka_feedback_linearization_controller {
class FrankaFeedbackLinearizationController : public controller_interface::MultiInterfaceController<
                                        franka_hw::FrankaModelInterface,
                                        hardware_interface::EffortJointInterface,
                                        franka_hw::FrankaStateInterface> {
public:
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration& period) override;

private: 
    pinocchio::Model model;
    pinocchio::Data data;
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;
    ros::Duration elapsed_time_;
    std::array<double, 7> initial_pose_{};

    int controlled_frame; 
    ros::Time beginTime; 

    double kp_delta_q;
    double kd_delta_dq;
    double kd_dq;

    double r_T;
    double p_T;
    double ya_T;
    double x_T;
    double y_T;
    double z_T;
    double x_T2;
    double y_T2;
    double z_T2;
    double x_T3;
    double y_T3;
    double z_T3;

    double dt1a; 
    double dt1b;
    double dt1c;
    double dt1d; 
    double dt2a; 
    double dt2b;
    double dt2c;
    double dt2d;
    double dt3a; 
    double dt3b;
    double dt3c;
    double dt3d;
    double dt4a; 
    double dt4b;
    double dt4c;
    double dt4d;
    double dt5a; 
    double dt5b;
    double dt5c;
    double dt5d;
    double dt7a; 
    double dt7b;
    double dt7c;
    double dt7d;
    double dt6a; 
    double dt6b;
    double dt6c;
    double dt6d;        

    bool secstart;
    bool thirdstart;
    
    double terminal_time;
    double terminal_time2;
    double terminal_time3;
    Eigen::Matrix<double, 6, 6> A{};
    Eigen::Matrix<double, 6, 6> Ainv{};
    Eigen::Matrix<double, 6, 1> Bx{};
    Eigen::Matrix<double, 6, 1> xx{};
    Eigen::Matrix<double, 6, 1> By{};
    Eigen::Matrix<double, 6, 1> xy{};
    Eigen::Matrix<double, 6, 1> Bz{};
    Eigen::Matrix<double, 6, 1> xz{};
    Eigen::Matrix<double, 6, 1> Br{};
    Eigen::Matrix<double, 6, 1> xr{};
    Eigen::Matrix<double, 6, 1> Bp{};
    Eigen::Matrix<double, 6, 1> xp{};
    Eigen::Matrix<double, 6, 1> Bya{};
    Eigen::Matrix<double, 6, 1> xya{};

    Eigen::Matrix<double, 6, 6> A2{};
    Eigen::Matrix<double, 6, 6> Ainv2{};
    Eigen::Matrix<double, 6, 6> A3{};
    Eigen::Matrix<double, 6, 6> Ainv3{};

    Eigen::Matrix<double, 7, 1> ddq_desired;
    Eigen::Matrix<double, 7, 1> torques;
    Eigen::Matrix<double, 7, 1> delta_q;
    Eigen::Matrix<double, 7, 1> delta_dq;
    Eigen::Matrix<double, 6, 7> dJ;
    Eigen::Matrix<double, 3, 7> Jp;
    Eigen::Matrix<double, 3, 7> dJp;

    ros::Publisher ee_measuredpos_pub;
    ros::Publisher ee_desiredpos_pub;
    ros::Publisher ee_measuredori_pub;
    ros::Publisher ee_desiredori_pub;
    ros::Publisher ori_error_pub; 
    ros::Publisher joint_angle_pub;
    ros::Publisher joint_velocity_pub;
    ros::Publisher desired_qt_pub;
    ros::Publisher torquepub;
    ros::Publisher vpub; 

    geometry_msgs::Vector3Stamped ee_desired_pos;
    geometry_msgs::Vector3Stamped ee_measured_pos;
    geometry_msgs::Vector3Stamped ee_desired_ori;
    geometry_msgs::Vector3Stamped ee_measured_ori;
    geometry_msgs::Vector3Stamped ee_ori_error;
    geometry_msgs::QuaternionStamped des_qt;  

    Eigen::Matrix<double, 1, 6> tpos {};
    Eigen::Matrix<double, 1, 6> tvel {};
    Eigen::Matrix<double, 1, 6> tacc {};
    Eigen::Matrix <double,6,1> X{};
    Eigen::Matrix <double,6,1> dX{};
    Eigen::Matrix <double,6,1> ddX{};  

    Eigen::Matrix<double, 6, 7> jacobian;
    Eigen::Matrix<double, 6, 1> error;
    Eigen::Matrix<double, 6, 1> derror;
    Eigen::Quaterniond Quaternion;
    double eta;
    double eta_d; 
    Eigen::Matrix<double, 3, 1> quat{};
    Eigen::Matrix<double, 3, 1> quat_d{};
    Eigen::Matrix<double, 3, 1> e_o{};

    Eigen::Matrix <double,7,1> vt{};
    Eigen::Matrix <double,7,1> ca{};
    Eigen::Matrix <double,7,1> cb{};
    Eigen::Matrix <double,7,1> cc{};
    







};
}