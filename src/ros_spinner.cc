#include <ros/ros.h>
#include <rtt/Activity.hpp>
#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/Logger.hpp>
#include <rtt/os/startstop.h>

using namespace RTT;

class RosSpinner : public RTT::TaskContext
{
private:
    RTT::OperationCaller<void(void)> spinOnce;
public:
RosSpinner(const std::string& name)
: TaskContext(name)
, spinOnce("spinOnce")
{
    this->requires("ros_init")->addOperationCaller(spinOnce);
}

bool configureHook()
{
    if(!ros::master::check())
    {
        log(Error) << "Ros Spinner wont be able to spin as ros is not started." << endlog();
        return false;
    }
    return true;
}

void updateHook()
{
    std::cout << "ros::spinOnce()" << '\n';
    //ros::spinOnce();
    spinOnce();
}

};

ORO_CREATE_COMPONENT(RosSpinner)
