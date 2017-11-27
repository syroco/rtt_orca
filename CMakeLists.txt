cmake_minimum_required(VERSION 3.1.0)
project(rtt_orca)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED orca)
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

orocos_use_package(ocl-taskbrowser)
orocos_use_package(ocl-deployment)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

orocos_service(RttOrcaLoggerService src/RttOrcaLoggerService.cc)
target_link_libraries(RttOrcaLoggerService ${catkin_LIBRARIES})

orocos_service(RttOrcaHelperService src/RttOrcaHelperService.cc)
target_link_libraries(RttOrcaHelperService ${catkin_LIBRARIES})

orocos_component(RttContact src/RttContact.cc)
target_link_libraries(RttContact ${catkin_LIBRARIES})

orocos_component(RttCartesianTask src/RttCartesianTask.cc)
target_link_libraries(RttCartesianTask ${catkin_LIBRARIES})

orocos_component(RttCartesianAccelerationPID src/RttCartesianAccelerationPID.cc)
target_link_libraries(RttCartesianAccelerationPID ${catkin_LIBRARIES})

orocos_component(RttRegularisationTask src/RttRegularisationTask.cc)
target_link_libraries(RttRegularisationTask ${catkin_LIBRARIES})

orocos_component(RttJointAccelerationTask src/RttJointAccelerationTask.cc)
target_link_libraries(RttJointAccelerationTask ${catkin_LIBRARIES})

orocos_component(RttWrenchTask src/RttWrenchTask.cc)
target_link_libraries(RttWrenchTask ${catkin_LIBRARIES})

orocos_component(RttDynamicsEquationConstraint src/RttDynamicsEquationConstraint.cc)
target_link_libraries(RttDynamicsEquationConstraint ${catkin_LIBRARIES})

orocos_component(RttJointLimitConstraint src/RttJointLimitConstraint.cc)
target_link_libraries(RttJointLimitConstraint ${catkin_LIBRARIES})

orocos_component(RttQPSolver src/RttQPSolver.cc)
target_link_libraries(RttQPSolver ${catkin_LIBRARIES})

install(FILES scripts/rtt_orca-test.ops DESTINATION lib/rtt_orca
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
                    GROUP_EXECUTE GROUP_READ)

orocos_generate_package()

