#include <ros/ros.h>
#include <orca/orca.h>
#include <orca_ros/orca_ros.h>

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>

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
    this->addProperty("base_frame",base_frame_);
    this->addProperty("robot_description",robot_description_);
    this->addProperty("controller_name",controller_name_);
    this->addProperty("robot_compensates_gravity",robot_compensates_gravity_);

    this->addProperty("joint_torque_max",joint_torque_max_);
    this->addProperty("joint_velocity_max",joint_velocity_max_);

    this->addPort("JointPosition",port_joint_position_in_).doc("Current joint positions");
    this->addPort("JointVelocity",port_joint_velocity_in_).doc("Current joint velocities");
    this->addPort("JointTorqueCommand",port_joint_torque_out_).doc("Command joint torques");

}

bool configureHook()
{
    std::cout << "robot_description_ " << robot_description_ << '\n';
    std::cout << "joint_torque_max_ " << joint_torque_max_ << '\n';
    std::cout << "joint_velocity_max_ " << joint_velocity_max_ << '\n';
    std::cout << "base_frame_ " << base_frame_ << '\n';
    // Create the robot model
    robot_kinematics_ = std::make_shared<orca::robot::RobotModel>();
    // Load the urdf file
    // robot_kinematics_->loadModelFromString(robot_description_);
    robot_kinematics_->loadModelFromFile("/home/hoarau/orca/examples/lwr.urdf");
    robot_kinematics_->print();

    // Set the base frame (for lwr its usually link_0)
    robot_kinematics_->setBaseFrame(base_frame_);

    const int ndof = robot_kinematics_->getNrOfDegreesOfFreedom();

    // Instanciate and ORCA Controller
    std::cout << "Robot is loaded, loading controller" <<'\n';

    auto robot_kinematics = std::make_shared<orca::robot::RobotModel>();
    robot_kinematics->loadModelFromFile("/home/hoarau/orca/examples/lwr.urdf");
    robot_kinematics->setBaseFrame(base_frame_);

    auto controller = std::make_shared<Controller>(
         controller_name_
        ,robot_kinematics
        ,ResolutionStrategy::OneLevelWeighted
        ,QPSolver::qpOASES
    );

    std::cout << "Controller is loaded" <<'\n';
    // Remve gravity from solution because lwr auto compensates gravity
    controller_->removeGravityTorquesFromSolution(robot_compensates_gravity_);

    // Create the regularization task that will make the robot PID around its first
    // position. This task is very optional
    joint_position_task_ = controller_->addTask<JointAccelerationTask>("JointPosTask");
    // Let's configure the internal PID
    Eigen::VectorXd P(ndof);
    P.setConstant(100);
    joint_position_task_->pid()->setProportionalGain(P);

    Eigen::VectorXd I(ndof);
    I.setConstant(1);
    joint_position_task_->pid()->setDerivativeGain(I);

    Eigen::VectorXd windupLimit(ndof);
    windupLimit.setConstant(10);
    joint_position_task_->pid()->setWindupLimit(windupLimit);

    Eigen::VectorXd D(ndof);
    D.setConstant(10);
    joint_position_task_->pid()->setDerivativeGain(D);
    // Set the weight for the task (its a regularization so it's small)
    joint_position_task_->setWeight(1.e-5);

    // Cartesian Task
    cart_task_ = controller_->addTask<CartesianTask>("CartTask_EE");
    cart_task_->setControlFrame(robot_kinematics_->getLinkNames().back()); // We want to control the link_7
    cart_task_->setRampDuration(0.5); // Activate immediately
    // Set the pose desired for the link_7
    Eigen::Affine3d cart_pos_ref;

    // Set the desired cartesian velocity to zero
    Vector6d cart_vel_ref;
    cart_vel_ref.setZero();

    // Set the desired cartesian velocity to zero
    Vector6d cart_acc_ref;
    cart_acc_ref.setZero();

    // Now set the servoing PID
    Vector6d cartP;
    cartP << 100, 100, 100, 10, 10, 10;
    cart_task_->servoController()->pid()->setProportionalGain(cartP);
    Vector6d cartD;
    cartD << 10, 10, 10, 1, 1, 1;
    cart_task_->servoController()->pid()->setDerivativeGain(cartD);

    // The joint torque limit constraint
    joint_torque_constraint_ = controller_->addConstraint<JointTorqueLimitConstraint>("JointTorqueLimit");

    joint_torque_constraint_->setLimits(-joint_torque_max_,joint_torque_max_);

    // Joint position limits are automatically extracted from the URDF model. Note that you can set them if you want. by simply doing jnt_pos_cstr->setLimits(jntPosMin,jntPosMax).
    joint_position_constraint_ = controller_->addConstraint<JointPositionLimitConstraint>("JointPositionLimit");

    // Joint velocity limits are usually given by the robot manufacturer
    joint_velocity_constraint_ = controller_->addConstraint<JointVelocityLimitConstraint>("JointVelocityLimit");
    joint_velocity_constraint_->setLimits(-joint_velocity_max_,joint_velocity_max_);

    controller_->globalRegularization()->euclidianNorm().setWeight(1.e-8);

    //RosGazeboModel gzrobot_ros_wrapper(gzrobot,robot_kinematics);
    //RosController controller_ros_wrapper(robot_name, controller); // TODO: take robot_kinematics
    //RosCartesianTask cart_task_ros_wrapper(robot_name, controller_->getName(), cart_task); // TODO: take robot_kinematics
    return true;
}

bool startHook()
{
    controller_->activateTasksAndConstraints();
}

void updateHook()
{

    RTT::FlowStatus fp = this->port_joint_position_in_.read(this->joint_position_in_);
    RTT::FlowStatus fv = this->port_joint_velocity_in_.read(this->joint_velocity_in_);

    // Return if not giving anything (might happend during startup)
    if(fp == RTT::NoData || fv == RTT::NoData)
    {
      //log(RTT::Error) << "Robot ports empty !" << endlog();
      return;
    }

    robot_kinematics_->setRobotState(this->joint_position_in_,this->joint_velocity_in_);


    auto current_time = RTT::os::TimeService::Instance()->getNSecs();
    double dt = this->getPeriod(); // the component period

    // Step the controller
    controller_->update(current_time,dt);

    // Method 2 : Break when no solution found
    if(controller_->solutionFound())
    {
        // NOTE : breaks are automatically disabled when sending a command
        // So no need to call gzrobot->setBrakes(false);
        port_joint_torque_out_.write( controller_->getJointTorqueCommand() );
    }
    else
    {
        // TODO : send a signal to KRL STOP2
        // No Solution found, breaking hard
        log(Error) << "No Solution found, setting to error" << endlog();
        this->error();
        return;
    }
}


private:
    std::string robot_description_;
    std::string base_frame_;
    std::string controller_name_ = "orca_controller";
    bool robot_compensates_gravity_ = true;
    std::shared_ptr<orca::robot::RobotModel> robot_kinematics_;
    std::shared_ptr<orca::optim::Controller> controller_;
    std::shared_ptr<orca::task::JointAccelerationTask> joint_position_task_;
    std::shared_ptr<orca::task::CartesianTask> cart_task_;
    std::shared_ptr<orca::constraint::JointTorqueLimitConstraint> joint_torque_constraint_;
    std::shared_ptr<orca::constraint::JointPositionLimitConstraint> joint_position_constraint_;
    std::shared_ptr<orca::constraint::JointVelocityLimitConstraint> joint_velocity_constraint_;

    Eigen::VectorXd joint_torque_max_;
    Eigen::VectorXd joint_velocity_max_;

    RTT::InputPort<Eigen::VectorXd> port_joint_position_in_;
    RTT::InputPort<Eigen::VectorXd> port_joint_velocity_in_;
    RTT::OutputPort<Eigen::VectorXd> port_joint_torque_out_;

    Eigen::VectorXd joint_torque_out_,
                    joint_position_in_,
                    joint_velocity_in_;
};

ORO_CREATE_COMPONENT(OrcaDemo)
