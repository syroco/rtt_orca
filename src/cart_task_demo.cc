#include <ros/ros.h>
#include <orca/orca.h>
#include <orca_ros/orca_ros.h>

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/OperationCaller.hpp>

using namespace orca::all;
using namespace orca_ros::all;

using namespace RTT;
using namespace RTT::os;

class CartTaskDemo : public RTT::TaskContext
{
public:
CartTaskDemo(const std::string& name)
: TaskContext(name)
, spinOnce("spinOnce")
{
    this->requires("ros_init")->addOperationCaller(spinOnce);
    this->provides("robot")->addProperty("name",robot_name_);
    this->provides("robot")->addProperty("base_frame",base_frame_);
    this->provides("robot")->addProperty("robot_description",robot_description_);
    this->provides("robot")->addProperty("robot_compensates_gravity",robot_compensates_gravity_);
    //orca/$(arg robot_name)/orca_controller/JointTorqueLimit
    this->provides("JointTorqueLimit")->addProperty("joint_torque_max",joint_torque_max_);
    this->provides("JointTorqueLimit")->addProperty("joint_velocity_max",joint_velocity_max_);

    this->addPort("JointPosition",port_joint_position_in_).doc("Current joint positions");
    this->addPort("JointVelocity",port_joint_velocity_in_).doc("Current joint velocities");
    this->addPort("JointTorqueCommand",port_joint_torque_out_).doc("Command joint torques");

}

bool configureHook()
{
    // Create the robot model
    robot_kinematics_ = std::make_shared<orca::robot::RobotModel>(robot_name_);
    // Load the urdf file
    robot_kinematics_->loadModelFromString(robot_description_);
    robot_kinematics_->print();
    // Set the base frame (for lwr its usually link_0)
    robot_kinematics_->setBaseFrame(base_frame_);

    const int ndof = robot_kinematics_->getNrOfDegreesOfFreedom();

    // Instanciate and ORCA Controller
    std::cout << "Robot is loaded, loading controller" <<'\n';

    controller_ = std::make_shared<Controller>(
          "orca_controller"
        ,robot_kinematics_
        ,ResolutionStrategy::OneLevelWeighted
        ,QPSolverImplType::qpOASES
    );

    std::cout << "Controller is loaded" <<'\n';
    // Remve gravity from solution because lwr auto compensates gravity
    controller_->removeGravityTorquesFromSolution(robot_compensates_gravity_);

    // Create the regularization task that will make the robot PID around its first
    // position. This task is very optional
    joint_position_task_ = controller_->addTask<JointAccelerationTask>("JointPosTask");
    // Let's configure the internal PID
    joint_position_task_->pid()->setProportionalGain( Eigen::VectorXd::Constant(ndof,100) );
    joint_position_task_->pid()->setDerivativeGain( Eigen::VectorXd::Constant(ndof,1) );
    joint_position_task_->pid()->setWindupLimit(Eigen::VectorXd::Constant(ndof,10));
    joint_position_task_->pid()->setDerivativeGain( Eigen::VectorXd::Constant(ndof,10));
    // Set the weight for the task (its a regularization so it's small)
    joint_position_task_->setWeight(1.e-5);

    auto cart_acc_pid = std::make_shared<CartesianAccelerationPID>("servo_controller");
    Vector6d P;
    P << 1000, 1000, 1000, 10, 10, 10;
    cart_acc_pid->pid()->setProportionalGain(P);
    Vector6d D;
    D << 100, 100, 100, 1, 1, 1;
    cart_acc_pid->pid()->setDerivativeGain(D);
    cart_acc_pid->setControlFrame("link_7");
    Eigen::Affine3d cart_pos_ref;
    cart_pos_ref.translation() = Eigen::Vector3d(0.3,-0.5,0.41); // x,y,z in meters
    cart_pos_ref.linear() = orca::math::quatFromRPY(M_PI,0,0).toRotationMatrix();
    Vector6d cart_vel_ref = Vector6d::Zero();
    Vector6d cart_acc_ref = Vector6d::Zero();
    cart_acc_pid->setDesired(cart_pos_ref.matrix(),cart_vel_ref,cart_acc_ref);
    
    auto cart_task = controller_->addTask<CartesianTask>("CartTask_EE");
    cart_task->setServoController(cart_acc_pid);

    // The joint torque limit constraint
    joint_torque_constraint_ = controller_->addConstraint<JointTorqueLimitConstraint>("JointTorqueLimit");

    joint_torque_constraint_->setLimits(-joint_torque_max_,joint_torque_max_);

    // Joint position limits are automatically extracted from the URDF model. Note that you can set them if you want. by simply doing jnt_pos_cstr->setLimits(jntPosMin,jntPosMax).
    joint_position_constraint_ = controller_->addConstraint<JointPositionLimitConstraint>("JointPositionLimit");

    // Joint velocity limits are usually given by the robot manufacturer
    joint_velocity_constraint_ = controller_->addConstraint<JointVelocityLimitConstraint>("JointVelocityLimit");
    joint_velocity_constraint_->setLimits(-joint_velocity_max_,joint_velocity_max_);

    controller_->globalRegularization()->euclidianNorm().setWeight(1.e-8);

    controller_ros_wrapper_ = std::make_shared<orca_ros::optim::RosController>(robot_kinematics_->getName(), controller_);
    cart_task_ros_wrapper_ = std::make_shared<orca_ros::task::RosCartesianTask>(robot_kinematics_->getName(), controller_->getName(), cart_task_); // TODO: take robot_kinematics
    return true;
}

bool startHook()
{
    controller_->activateTasksAndConstraints();
    return true;
}

void updateHook()
{
    RTT::FlowStatus fp = this->port_joint_position_in_.read(this->joint_position_in_);
    RTT::FlowStatus fv = this->port_joint_velocity_in_.read(this->joint_velocity_in_);

    // Return if not giving anything (might happend during startup)
    if(fp == RTT::NoData || fv == RTT::NoData)
    {
        static bool p = true;
        if(p)
        {
            p = false;
            log(RTT::Info) << getName() << " ------> waiting for (q,dq)" << endlog();
        }
      //log(RTT::Info) << "Robot ports empty !" << endlog();
      return;
    }

    static bool p = true;
    if(p)
    {
        p = false;
        log(RTT::Info) << getName() << " ------> First (q,dq) received !" << endlog();
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
    spinOnce();
}


private:
    RTT::OperationCaller<void(void)> spinOnce;
    std::string robot_description_;
    std::string robot_name_;
    std::string base_frame_;
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

    std::shared_ptr<orca_ros::optim::RosController> controller_ros_wrapper_;
    std::shared_ptr<orca_ros::task::RosCartesianTask> cart_task_ros_wrapper_;
};

ORO_CREATE_COMPONENT(CartTaskDemo)
