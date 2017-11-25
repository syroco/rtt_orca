cmake_minimum_required(VERSION 3.1.0)
project(rtt_orca)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin QUIET)
find_package(orca REQUIRED)
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

orocos_use_package(ocl-taskbrowser)
orocos_use_package(ocl-deployment)

include_directories(
    include
    ${orca_INCLUDE_DIRS}
    ${OROCOS-RTT_INCLUDE_DIRS}
)

orocos_service(RttOrcaLoggerService src/RttOrcaLoggerService.cc)
target_link_libraries(RttOrcaLoggerService ${orca_LIBRARIES})

orocos_service(RttOrcaRegisterService src/RttOrcaRegisterService.cc)
target_link_libraries(RttOrcaRegisterService ${orca_LIBRARIES})

orocos_component(RttContact src/RttContact.cc)
target_link_libraries(RttContact ${orca_LIBRARIES})

orocos_component(RttCartesianTask src/RttCartesianTask.cc)
target_link_libraries(RttCartesianTask ${orca_LIBRARIES})

orocos_component(RttRegularisationTask src/RttRegularisationTask.cc)
target_link_libraries(RttRegularisationTask ${orca_LIBRARIES})

orocos_component(RttJointAccelerationTask src/RttJointAccelerationTask.cc)
target_link_libraries(RttJointAccelerationTask ${orca_LIBRARIES})

orocos_component(RttWrenchTask src/RttWrenchTask.cc)
target_link_libraries(RttWrenchTask ${orca_LIBRARIES})

orocos_component(RttDynamicsEquationConstraint src/RttDynamicsEquationConstraint.cc)
target_link_libraries(RttDynamicsEquationConstraint ${orca_LIBRARIES})

orocos_component(RttQPSolver src/RttQPSolver.cc)
target_link_libraries(RttQPSolver ${orca_LIBRARIES})

orocos_generate_package()

