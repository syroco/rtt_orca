#include <rtt/plugin/Plugin.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Activity.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt/Logger.hpp>
#include <rtt/os/startstop.h>
#include <ros/ros.h>

using namespace RTT;

void loadROSService()
{
  RTT::Service::shared_ptr ros = RTT::internal::GlobalService::Instance()->provides("ros_init");

  ros->addOperation("getNodeName", &ros::this_node::getName,OwnThread)
	  .doc("Return full name of ROS node.");
  ros->addOperation("getNamespace", &ros::this_node::getNamespace,OwnThread)
	  .doc("Return ROS node namespace.");
  ros->addOperation("spinOnce", &ros::spinOnce,OwnThread)
	  .doc("Spin once.");
}

extern "C" {
  bool loadRTTPlugin(RTT::TaskContext* c){

    // TODO: Check for existing conflicting rtt_rosnode
    // RTT::Service::shared_ptr ros = RTT::internal::GlobalService::Instance()->provides("ros");

    // Initialize ROS if necessary
    if (!ros::isInitialized()) {
      std::cout << "\x1B[34m[ROS] ----> Initializing ROS node...\x1B[0m" << std::endl;

      int argc = __os_main_argc();
      char ** argv = __os_main_argv();

      // copy the argv array of C strings into a std::vector<char *>
      // Rationale: ros::init(int &argc, char **argv) removes some of the
      // command line arguments and rearranges the remaining ones in the argv
      // vector.
      // See https://github.com/orocos/rtt_ros_integration/issues/54
      std::vector<char *> argvv(argv, argv + argc);
      assert(argvv.size() == argc);
      ros::init(argc, argvv.data(), "rtt", ros::init_options::AnonymousName);
      argvv.resize(argc);

      if (ros::master::check()) {
        ros::start();
      } else {
        log(Warning) << "[ROS] 'roscore' is not running: no ROS functions will be available." << endlog();
        ros::shutdown();
        return true;
      }

	  // Register new operations in global ros Service
	  loadROSService();
    }
    else
    {
        log(Warning) << "[ROS] Ros is already initialized" << endlog();
    }
    return true;
  }

  std::string getRTTPluginName () {
    return "rosinit";
  }

  std::string getRTTTargetName () {
    return OROCOS_TARGET_NAME;
  }
}
