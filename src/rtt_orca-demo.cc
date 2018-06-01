#include <ros/ros.h>
#include <orca/orca.h>
#include <orca_ros/orca_ros.h>

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>

#include <rtt_rosclock/rtt_rosclock.h>

using namespace orca::all;
using namespace orca_ros::all;

using namespace RTT;
using namespace RTT::os;

class OrcaDemo : public RTT::TaskContext
{
public:
OrcaDemo(const std::string& name)
: TaskContext(name)
{
    this->addProperty("max_n_missed",max_n_missed);
    this->addPort("JointPosition",port_joint_position_in_).doc("Current joint positions");
    this->addPort("JointVelocity",port_joint_velocity_in_).doc("Current joint velocities");
    this->addPort("JointTorqueCommand",port_joint_torque_out_).doc("Command joint torques");
    this->addPort("robot_state",port_state_msg_).doc("Command joint torques from ROS");
    this->addPort("JointTorqueCommandFromROS",port_jnt_trq_cmd_).doc("Command joint torques from ROS");

}

bool configureHook()
{
    Eigen::Affine3d id(Eigen::Affine3d::Identity());

    tf::transformEigenToMsg(id, state_msg_.world_to_base_transform);

    Eigen::Matrix< double, 6, 1 > base_vel;
    base_vel.setZero();
    tf::twistEigenToMsg(base_vel, state_msg_.base_velocity);


    Eigen::Vector3d gravity = Eigen::Vector3d(0,0,-9.81);
    tf::vectorEigenToMsg(gravity, state_msg_.gravity);

    return true;
}

bool startHook()
{
    controller_->activateTasksAndConstraints();
}

void updateHook()
{
    static int n_missed = 0;

    RTT::FlowStatus fp = this->port_joint_position_in_.read(this->joint_position_in_);
    RTT::FlowStatus fv = this->port_joint_velocity_in_.read(this->joint_velocity_in_);

    // Return if not giving anything (might happend during startup)
    if(fp == RTT::NoData || fv == RTT::NoData)
    {
      //log(RTT::Error) << "Robot ports empty !" << endlog();
      return;
    }

    RTT::FlowStatus ft = port_jnt_trq_cmd_.read(jnt_trq_cmd_);
    if(ft == RTT::NoData)
    {
      //log(RTT::Error) << "Robot ports empty !" << endlog();
      return;
    }
    if(ft == RTT::NewData)
    {
        n_missed = 0;
        if(trq_cmd_out.size() != jnt_trq_cmd_.joint_torque_command.size())
            trq_cmd_out.resize(jnt_trq_cmd_.joint_torque_command.size());
        trq_cmd_out = Eigen::Map<const Eigen::VectorXd>(jnt_trq_cmd_.joint_torque_command.data(),jnt_trq_cmd_.joint_torque_command.size());
        port_joint_torque_out_.write(trq_cmd_out);
    }
    else
    {
        // Old Data
        n_missed++;
        if(n_missed < max_n_missed)
        {
            port_joint_torque_out_.write(trq_cmd_out);
        }
    }


    //Eigen::VectorXd::Map(state_msg_.joint_external_torques.data(),state_msg_.joint_external_torques.size()) = gz_model_->getJointExternalTorques();
    //Eigen::VectorXd::Map(state_msg_.joint_measured_torques.data(),state_msg_.joint_measured_torques.size()) = gz_model_->getJointMeasuredTorques();

    if(state_msg_.joint_positions.size() != joint_position_in_.size())
    {
        state_msg_.joint_positions.resize(joint_position_in_.size());
    }

    if(state_msg_.joint_velocities.size() != joint_velocity_in_.size())
    {
        state_msg_.joint_velocities.resize(joint_velocity_in_.size());
    }

    Eigen::VectorXd::Map(state_msg_.joint_positions.data(),state_msg_.joint_positions.size()) = joint_position_in_;
    Eigen::VectorXd::Map(state_msg_.joint_velocities.data(),state_msg_.joint_velocities.size()) = joint_velocity_in_;

    state_msg_.header.stamp = rtt_rosclock::host_now();
    port_state_msg_.write( state_msg_ );
}


private:
    std::string robot_description_;
    std::string base_frame_;
    std::string controller_name_ = "orca_controller";
    bool robot_compensates_gravity_ = true;
    std::shared_ptr<orca::robot::RobotDynTree> robot_kinematics_;
    std::shared_ptr<orca::optim::Controller> controller_;
    std::shared_ptr<orca::task::JointAccelerationTask> joint_position_task_;
    std::shared_ptr<orca::task::CartesianTask> cart_task_;
    std::shared_ptr<orca::constraint::JointTorqueLimitConstraint> joint_torque_constraint_;
    std::shared_ptr<orca::constraint::JointPositionLimitConstraint> joint_position_constraint_;
    std::shared_ptr<orca::constraint::JointVelocityLimitConstraint> joint_velocity_constraint_;
    std::shared_ptr<orca_ros::optim::RosController> controller_ros_wrapper_;
    std::shared_ptr<orca_ros::task::RosCartesianTask> cart_task_ros_wrapper_;
    std::shared_ptr<orca_ros::robot::RosRobotDynTree> robot_ros_wrapper_;

    int max_n_missed = 50;

    Eigen::VectorXd joint_torque_max_;
    Eigen::VectorXd joint_velocity_max_;

    RTT::InputPort<Eigen::VectorXd> port_joint_position_in_;
    RTT::InputPort<Eigen::VectorXd> port_joint_velocity_in_;
    RTT::OutputPort<Eigen::VectorXd> port_joint_torque_out_;

    orca_ros::RobotState state_msg_;
    RTT::OutputPort<orca_ros::RobotState> port_state_msg_;
    orca_ros::JointTorqueCommand jnt_trq_cmd_;
    Eigen::VectorXd trq_cmd_out;
    RTT::InputPort<orca_ros::JointTorqueCommand> port_jnt_trq_cmd_;

    Eigen::VectorXd joint_torque_out_,
                    joint_position_in_,
                    joint_velocity_in_;
};

ORO_CREATE_COMPONENT(OrcaDemo)
