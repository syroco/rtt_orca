#pragma once

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/Operation.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/Property.hpp>
#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

#include <orca/orca.h>

namespace rtt_orca
{
namespace common
{
    class RttTaskCommon: public RTT::TaskContext
    {
    public:
        RttTaskCommon(RTT::TaskContext* owner,orca::common::TaskCommon* comm,const std::string& name)
        : RTT::TaskContext(name)
        , robot_(comm->robot())
        , comm_(*comm)
        {
            comm->setName(name);
            
            owner->provides("robot_model")->addOperation("loadModelFromFile",&RttTaskCommon::loadRobotModel,this , RTT::OwnThread);
            owner->provides("robot_model")->addOperation("print", &orca::robot::RobotDynTree::print, &robot_ , RTT::OwnThread);
            owner->provides("robot_model")->addOperation("setBaseFrame", &orca::robot::RobotDynTree::setBaseFrame, &robot_ , RTT::OwnThread);
            owner->provides("robot_model")->addOperation("setGravity", &RttTaskCommon::setGravity, this , RTT::OwnThread);
            owner->provides("robot_model")->provides("state")->addPort("JointPosition",port_jnt_pos_in_);
            owner->provides("robot_model")->provides("state")->addPort("JointVelocity",port_jnt_vel_in_);
            owner->provides("robot_model")->provides("state")->addPort("WorldToBase",port_world_to_base_in_);
            owner->provides("robot_model")->provides("state")->addPort("BaseVelocity",port_base_vel_in_);
            owner->provides("robot_model")->provides("state")->addPort("Gravity",port_gravity_in_);
            
            owner->addOperation("desactivate",&orca::common::TaskCommon::desactivate,comm,RTT::OwnThread);
            owner->addOperation("activate",&orca::common::TaskCommon::activate,comm,RTT::OwnThread);
            owner->addOperation("isActivated",&orca::common::TaskCommon::isActivated,comm,RTT::OwnThread);
            owner->addOperation("insertInProblem",&orca::common::TaskCommon::insertInProblem,comm,RTT::OwnThread);
            owner->addOperation("removeFromProblem",&orca::common::TaskCommon::removeFromProblem,comm,RTT::OwnThread);
            owner->addOperation("isInsertedInProblem",&orca::common::TaskCommon::isInsertedInProblem,comm,RTT::OwnThread);
            owner->addOperation("isInitialized",&orca::common::TaskCommon::isInitialized,comm,RTT::OwnThread);
            owner->addOperation("connectToRobot",&RttTaskCommon::connectToRobot,this,RTT::OwnThread);
            
            owner->addOperation("print",&orca::common::TaskCommon::print,comm,RTT::OwnThread);
            
            // Waiting for OROCOS to support fixed size vectors
            gravity_in_.setZero(3);
            base_vel_in_.setZero(6);
            world_to_base_in_.resize(4,4);
            world_to_base_in_.setIdentity();
        }

        bool connectToRobot(const std::string& robot_comp)
        {
            RTT::Logger::In in("connectToRobot");
            RTT::TaskContext * robot = getPeer(robot_comp);
            
            if(!robot)
            {
                RTT::log(RTT::Error) << "Could not find robot peer. Did you forget to call \n\n   connectPeers(\"" << getName() << "\",\"" << robot_comp << "\")" << RTT::endlog();
                return false;
            }
            
            auto pr = robot->provides("state");
            
            if(!pr)
            {
                RTT::log(RTT::Error) << robot_comp << " does not seem to provide a 'state' service" << RTT::endlog();
                return false;
            }

            auto port = pr->getPort("JointPosition");
            if(port)
                port_jnt_pos_in_.connectTo(port);
            
            port = pr->getPort("JointVelocity");
            if(port)
                port_jnt_vel_in_.connectTo(port);
            
            port = pr->getPort("WorldToBase");
            if(port)
                port_jnt_vel_in_.connectTo(port);
            
            port = pr->getPort("BaseVelocity");
            if(port)
                port_jnt_vel_in_.connectTo(port);
            
            port = pr->getPort("Gravity");
            if(port)
                port_jnt_vel_in_.connectTo(port);

            return true;
        }
        
        void setGravity(const Eigen::VectorXd& grav)
        {
            if(grav.size() != 3)
            {
                RTT::log(RTT::Error) << "Wrong vector size. Provided " << grav.size() << ", expected 3" << RTT::endlog();
                return;
            }
            robot_data_helper_.eigRobotState.gravity = grav.head<3>();
            gravity_in_ = grav;
            robot_.setGravity(gravity_in_);
        }
        
        bool updateRobotModel()
        {
            RTT::FlowStatus fspos = port_jnt_pos_in_.readNewest(robot_data_helper_.eigRobotState.jointPos);
            RTT::FlowStatus fsvel = port_jnt_vel_in_.readNewest(robot_data_helper_.eigRobotState.jointVel);
            RTT::FlowStatus fswtb = port_world_to_base_in_.readNewest(world_to_base_in_);
            RTT::FlowStatus fsbve = port_base_vel_in_.readNewest(base_vel_in_);
            RTT::FlowStatus fsgra = port_gravity_in_.readNewest(gravity_in_);
            
            if(fsgra == RTT::NewData)
                robot_data_helper_.eigRobotState.gravity = gravity_in_.head<3>();
                
            if(fsbve == RTT::NewData)
                robot_data_helper_.eigRobotState.baseVel = base_vel_in_.head<6>();
                
            if(fswtb == RTT::NewData)
                robot_data_helper_.eigRobotState.world_H_base = world_to_base_in_.block<4,4>(0,0);
            
            if(fspos == RTT::NoData || fsvel == RTT::NoData)
            {
                return false;
            }
            
            if(fspos != RTT::NoData
            && fsvel != RTT::NoData
            && fswtb != RTT::NoData
            && fsbve != RTT::NoData
            && fsgra != RTT::NoData)
            {
                robot_.setRobotState(robot_data_helper_.eigRobotState.world_H_base
                            ,robot_data_helper_.eigRobotState.jointPos
                            ,robot_data_helper_.eigRobotState.baseVel
                            ,robot_data_helper_.eigRobotState.jointVel
                            ,robot_data_helper_.eigRobotState.gravity
                                                    );
                return true;
            }
            
            if(fspos != RTT::NoData
            && fsvel != RTT::NoData
            && fswtb == RTT::NoData
            && fsbve == RTT::NoData
            && fsgra == RTT::NoData)
            {
                robot_.setRobotState(robot_data_helper_.eigRobotState.jointPos
                            ,robot_data_helper_.eigRobotState.jointVel);
                return true;
            }
            
            if(fspos != RTT::NoData
            && fsvel != RTT::NoData
            && fswtb == RTT::NoData
            && fsbve == RTT::NoData
            && fsgra != RTT::NoData)
            {
                robot_.setRobotState(robot_data_helper_.eigRobotState.jointPos
                            ,robot_data_helper_.eigRobotState.jointVel
                            ,robot_data_helper_.eigRobotState.gravity);
                return true;
            }
            
            std::cout << "fspos = " << fspos << std::endl;
            std::cout << "fsvel = " << fsvel << std::endl;
            std::cout << "fswtb = " << fswtb << std::endl;
            std::cout << "fsbve = " << fsbve << std::endl;
            std::cout << "fsgra = " << fsgra << std::endl;
            return false;
        }
        
        bool loadRobotModel(const std::string& file_url)
        {
            if(comm_.loadRobotModel(file_url))
                robot_data_helper_.resize(robot_.getRobotModel());
            else
                throw std::runtime_error("Could not load robot model");
            return true;
        }

    private:
        RTT::InputPort<Eigen::VectorXd> port_jnt_pos_in_;
        RTT::InputPort<Eigen::VectorXd> port_jnt_vel_in_;
        RTT::InputPort<Eigen::MatrixXd> port_world_to_base_in_;
        RTT::InputPort<Eigen::VectorXd> port_base_vel_in_;
        RTT::InputPort<Eigen::VectorXd> port_gravity_in_;
        
        Eigen::VectorXd gravity_in_;
        Eigen::VectorXd base_vel_in_;
        Eigen::MatrixXd world_to_base_in_;
        
        orca::robot::RobotDynTree& robot_;
        orca::robot::RobotDataHelper robot_data_helper_;
        orca::common::TaskCommon& comm_;
    };
} // namespace common
} // namespace rtt_orca
