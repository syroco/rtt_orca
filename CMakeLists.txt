cmake_minimum_required(VERSION 3.1.0)
project(rtt_orca)
find_package(orca REQUIRED)
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

orocos_component(rtt_orca-demo src/rtt_orca-demo.cc)
target_link_libraries(rtt_orca-demo ${catkin_LIBRARIES} orca::orca)
#
# install(FILES scripts/rtt_orca-demo.ops DESTINATION lib/rtt_orca
#     PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
#                     GROUP_EXECUTE GROUP_READ)

orocos_generate_package()
