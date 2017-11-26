cmake_minimum_required(VERSION 3.1.0)
project(rtt_orca)

find_package(orca REQUIRED)
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

orocos_find_package(orca REQUIRED)

orocos_use_package(ocl-taskbrowser)
orocos_use_package(ocl-deployment)

include_directories(
    include
)

orocos_service(RttOrcaLoggerService src/RttOrcaLoggerService.cc)
target_link_libraries(RttOrcaLoggerService orca::orca)

orocos_service(RttOrcaRegisterService src/RttOrcaRegisterService.cc)
target_link_libraries(RttOrcaRegisterService orca::orca)

orocos_component(RttContact src/RttContact.cc)
target_link_libraries(RttContact orca::orca)

orocos_component(RttCartesianTask src/RttCartesianTask.cc)
target_link_libraries(RttCartesianTask orca::orca)

orocos_component(RttRegularisationTask src/RttRegularisationTask.cc)
target_link_libraries(RttRegularisationTask orca::orca)

orocos_component(RttJointAccelerationTask src/RttJointAccelerationTask.cc)
target_link_libraries(RttJointAccelerationTask orca::orca)

orocos_component(RttWrenchTask src/RttWrenchTask.cc)
target_link_libraries(RttWrenchTask orca::orca)

orocos_component(RttDynamicsEquationConstraint src/RttDynamicsEquationConstraint.cc)
target_link_libraries(RttDynamicsEquationConstraint orca::orca)

orocos_component(RttQPSolver src/RttQPSolver.cc)
target_link_libraries(RttQPSolver orca::orca)

install(FILES scripts/rtt_orca-test.ops DESTINATION lib/rtt_orca
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
                    GROUP_EXECUTE GROUP_READ)

orocos_generate_package()

