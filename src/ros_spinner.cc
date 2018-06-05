#include <ros/ros.h>
#include <rtt/Activity.hpp>
#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/os/startstop.h>

using namespace RTT;

class RosSpinner : public RTT::TaskContext
{
public:
RosSpinner(const std::string& name)
: TaskContext(name)
{}

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
    return ;
    // std::cout << "ros::spinOnce()" << '\n';
    ros::spinOnce();
}

};

ORO_CREATE_COMPONENT(RosSpinner)
