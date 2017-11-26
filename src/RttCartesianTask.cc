#include <rtt_orca/task/RttGenericTask.h>

namespace rtt_orca
{
namespace task
{
    class RttCartesianTask: public orca::task::CartesianTask, public task::RttGenericTask
    {
    public:
        RttCartesianTask(const std::string& name)
        : task::RttGenericTask(this,this,name)
        {
            this->addOperation("setBaseFrame",&orca::task::CartesianTask::setBaseFrame,this,RTT::OwnThread);
            this->addOperation("setControlFrame",&orca::task::CartesianTask::setControlFrame,this,RTT::OwnThread);
            this->addOperation("getBaseFrame",&orca::task::CartesianTask::getBaseFrame,this,RTT::OwnThread);
            this->addOperation("getControlFrame",&orca::task::CartesianTask::getControlFrame,this,RTT::OwnThread);
            
            this->provides("input")->addPort("CartesianAcceleration",port_cartesian_acceleration_des_);
        }

        void updateHook()
        {
            if(this->updateRobotModel())
            {
                if(port_cartesian_acceleration_des_.readNewest(input_cmd_) == RTT::NewData)
                {
                    orca::task::CartesianTask::setDesired(input_cmd_);
                }
                orca::task::CartesianTask::update();
            }
        }

    protected:
        RTT::InputPort<Eigen::Matrix<double,6,1> > port_cartesian_acceleration_des_;
        Eigen::Matrix<double,6,1> input_cmd_;
    };
}
}

ORO_CREATE_COMPONENT(rtt_orca::task::RttCartesianTask)
