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
    ros::Duration MessageTime;
    ros::Time endTime;

    double kp_delta_q;
    double kd_delta_dq;
    double kd_dq;

    double r_T;
    double p_T;
    double ya_T;
    double x_T;
    double y_T;
    double z_T;
    
    double terminal_time;
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

    geometry_msgs::Vector3Stamped ee_desired_pos;
    geometry_msgs::Vector3Stamped ee_measured_pos;
    geometry_msgs::Vector3Stamped ee_desired_ori;
    geometry_msgs::Vector3Stamped ee_measured_ori;
    geometry_msgs::Vector3Stamped ee_ori_error;

    Eigen::Matrix<double, 1, 6> tpos {};
    Eigen::Matrix<double, 1, 6> tvel {};
    Eigen::Matrix<double, 1, 6> tacc {};
    Eigen::Matrix <double,6,1> X{};
    Eigen::Matrix <double,6,1> dX{};
    Eigen::Matrix <double,6,1> ddX{};  

    Eigen::Matrix<double, 6, 7> jacobian;
    Eigen::Matrix<double, 6, 1> error;
    Eigen::Quaterniond Quaternion;





};
}