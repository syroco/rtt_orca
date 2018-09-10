cmake_minimum_required(VERSION 3.1.0)
project(rtt_orca)

find_package(catkin REQUIRED rtt_ros
                            orca_ros
                            rtt_roscomm
                            rtt_rosclock
                            eigen_conversions
                        )

ros_generate_rtt_typekit(orca_ros)
ros_generate_rtt_service_proxies(orca_ros)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

orocos_plugin(ros_init_plugin src/ros_init_plugin.cc)
target_link_libraries(ros_init_plugin ${catkin_LIBRARIES})

orocos_component(ros_spinner src/ros_spinner.cc)
target_link_libraries(ros_spinner ${catkin_LIBRARIES})

orocos_component(joint_torque_streamer src/joint_torque_streamer.cc)
target_link_libraries(joint_torque_streamer ${catkin_LIBRARIES})


orocos_component(cart_task_demo src/cart_task_demo.cc)
target_link_libraries(cart_task_demo ${catkin_LIBRARIES})
#
# install(FILES scripts/rtt_orca-demo.ops DESTINATION lib/rtt_orca
#     PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
#                     GROUP_EXECUTE GROUP_READ)

orocos_generate_package()
