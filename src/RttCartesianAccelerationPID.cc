#include <rtt_orca/common/RttTaskCommon.h>

namespace rtt_orca
{
namespace common
{
    class RttCartesianAccelerationPID: public orca::common::CartesianAccelerationPID, public RttTaskCommon
    {
    public:
        RttCartesianAccelerationPID(const std::string& name)
        : RttTaskCommon(this,this,name)
        {
            this->addOperation("setBaseFrame",&orca::common::CartesianAccelerationPID::setBaseFrame,this,RTT::OwnThread);
            this->addOperation("setControlFrame",&orca::common::CartesianAccelerationPID::setControlFrame,this,RTT::OwnThread);
            this->addOperation("getBaseFrame",&orca::common::CartesianAccelerationPID::getBaseFrame,this,RTT::OwnThread);
            this->addOperation("getControlFrame",&orca::common::CartesianAccelerationPID::getControlFrame,this,RTT::OwnThread);
            
            this->provides("pid")->addOperation("setProportionalGain",&RttCartesianAccelerationPID::setProportionalGain,this,RTT::OwnThread);
            this->provides("pid")->addOperation("setDerivativeGain",&RttCartesianAccelerationPID::setDerivativeGain,this,RTT::OwnThread);
            
            this->addOperation("setDesired",&orca::common::CartesianAccelerationPID::setDesired,this,RTT::OwnThread);
            this->addOperation("setDesiredPosition",&RttCartesianAccelerationPID::setDesiredPosition,this,RTT::OwnThread);
            this->provides("output")->addPort("CartesianAcceleration",port_cartesian_acceleration_cmd_);
        }

        void setDesiredPosition(const std::vector<double>& pos,const std::vector<double>& rpy)
        {
            if(pos.size() != 3)
            {
                RTT::log(RTT::Error) << "Size of vector should be 3, provided " << pos.size() << RTT::endlog();
                return;
            }
            if(rpy.size() != 3)
            {
                RTT::log(RTT::Error) << "Size of vector should be 3, provided " << rpy.size() << RTT::endlog();
                return;
            }
            Eigen::Vector3d position(pos.data());
            
            Eigen::Affine3d cart_pos_ref;
            cart_pos_ref.translation() = position;
            Eigen::Quaterniond quat;
            quat = Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())
                 * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY())
                 * Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ());
            cart_pos_ref.linear() = quat.toRotationMatrix();

            Eigen::Matrix<double,6,1> cart_vel_ref;
            cart_vel_ref.setZero();
            Eigen::Matrix<double,6,1> cart_acc_ref;
            cart_acc_ref.setZero();
            
            orca::common::CartesianAccelerationPID::setDesired(cart_pos_ref.matrix(),cart_vel_ref,cart_acc_ref);
        }

        void setProportionalGain(const std::vector<double>& v)
        {
            if(v.size() != 6)
            {
                RTT::log(RTT::Error) << "Size of vector should be 6, provided " << v.size() << RTT::endlog();
                return;
            }
            Eigen::Matrix<double,6,1> gain(v.data());
            this->pid().setProportionalGain(gain);
        }

        void setDerivativeGain(const std::vector<double>& v)
        {
            if(v.size() != 6)
            {
                RTT::log(RTT::Error) << "Size of vector should be 6, provided " << v.size() << RTT::endlog();
                return;
            }
            Eigen::Matrix<double,6,1> gain(v.data());
            this->pid().setDerivativeGain(gain);
        }

        void updateHook()
        {
            if(this->updateRobotModel())
            {
                orca::common::CartesianAccelerationPID::update();
                port_cartesian_acceleration_cmd_.write( orca::common::CartesianAccelerationPID::getCommand() );
            }
        }

    protected:
        RTT::OutputPort<Eigen::Matrix<double,6,1> > port_cartesian_acceleration_cmd_;
    };
}
}

ORO_CREATE_COMPONENT(rtt_orca::common::RttCartesianAccelerationPID)
