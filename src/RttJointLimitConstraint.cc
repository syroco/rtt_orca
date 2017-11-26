#include <rtt_orca/constraint/RttGenericConstraint.h>

namespace rtt_orca
{
namespace constraint
{
    class RttJointTorqueLimitConstraint: public orca::constraint::JointTorqueLimitConstraint, public constraint::RttGenericConstraint
    {
    public:
        RttJointTorqueLimitConstraint(const std::string& name)
        : constraint::RttGenericConstraint(this,this,name)
        {
            this->addOperation("setLimits",&JointTorqueLimitConstraint::setLimits,this,RTT::OwnThread);
        }

        void updateHook()
        {
            if(this->updateRobotModel())
                orca::constraint::JointTorqueLimitConstraint::update();
        }
    };
    
    class RttJointAccelerationLimitConstraint: public orca::constraint::JointAccelerationLimitConstraint, public constraint::RttGenericConstraint
    {
    public:
        RttJointAccelerationLimitConstraint(const std::string& name)
        : constraint::RttGenericConstraint(this,this,name)
        {
            this->addOperation("setLimits",&JointAccelerationLimitConstraint::setLimits,this,RTT::OwnThread);
        }

        void updateHook()
        {
            orca::constraint::JointAccelerationLimitConstraint::update();
        }
    };
    
    class RttJointVelocityLimitConstraint: public orca::constraint::JointVelocityLimitConstraint, public constraint::RttGenericConstraint
    {
    public:
        RttJointVelocityLimitConstraint(const std::string& name)
        : constraint::RttGenericConstraint(this,this,name)
        {
            this->addOperation("setLimits",&JointVelocityLimitConstraint::setLimits,this,RTT::OwnThread);
            this->addOperation("setHorizon",&JointVelocityLimitConstraint::setHorizon,this,RTT::OwnThread);
        }

        void updateHook()
        {
            if(this->updateRobotModel())
                orca::constraint::JointVelocityLimitConstraint::update();
        }
    };
    
    class RttJointPositionLimitConstraint: public orca::constraint::JointPositionLimitConstraint, public constraint::RttGenericConstraint
    {
    public:
        RttJointPositionLimitConstraint(const std::string& name)
        : constraint::RttGenericConstraint(this,this,name)
        {
            this->addOperation("setLimits",&JointPositionLimitConstraint::setLimits,this,RTT::OwnThread);
            this->addOperation("setHorizon",&JointPositionLimitConstraint::setHorizon,this,RTT::OwnThread);
        }

        void updateHook()
        {
            if(this->updateRobotModel())
                orca::constraint::JointPositionLimitConstraint::update();
        }
    };
}
}

ORO_LIST_COMPONENT_TYPE(rtt_orca::constraint::RttJointTorqueLimitConstraint)
ORO_LIST_COMPONENT_TYPE(rtt_orca::constraint::RttJointAccelerationLimitConstraint)
ORO_LIST_COMPONENT_TYPE(rtt_orca::constraint::RttJointVelocityLimitConstraint)
ORO_LIST_COMPONENT_TYPE(rtt_orca::constraint::RttJointPositionLimitConstraint)
ORO_CREATE_COMPONENT_LIBRARY()
