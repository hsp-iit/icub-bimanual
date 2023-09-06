cmake_minimum_required(VERSION 2.8.9)					                            # Minimum cmake requirement

project(icub-bimanual)							                            # ${PROJECT_NAME}

# cmake -DCMAKE_BUILD_TYPE=Release ..

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)                                                               # Release = Optimised
endif()
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
################################## Necessary packages to compile ###################################
find_package(Eigen3 REQUIRED)
#find_package(ICUB REQUIRED)                                                                        # Links below to ICUB::iKin
find_package(iDynTree REQUIRED)                                                                     # Links below to iDynTree
find_package(YARP 3.3.0 REQUIRED)                                                                   # Links below to ${YARP_LIBRARIES}

######################################## iCub related stuff ########################################
find_package(ICUBcontrib REQUIRED)                                                                  # Mandatory
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})                                           # Extend the current search path to load helpers
include(ICUBcontribHelpers)                                                                         # I don't know what this does
include(ICUBcontribOptions)
#icubcontrib_set_default_prefix()
icubcontrib_add_uninstall_target()					                            # Enables uninstall
#################################### Other files to be built #######################################
add_subdirectory(gazebo)                                                                            # Location of other CMakeLists
add_subdirectory(interface)                                                                         # Location & definition of thrift interface
include_directories(include)                                                                        # Location of header files

###################################### Walking Controller Thrift ###################################

set(generated_libs_dir "${CMAKE_CURRENT_SOURCE_DIR}")

yarp_idl_to_dir(INPUT_FILES thrifts/WalkingControllers/YarpUtilities/HumanState.thrift
                OUTPUT_DIR ${generated_libs_dir}
                SOURCES_VAR sources
                HEADERS_VAR headers
                INCLUDE_DIRS_VAR include_dirs)

  
#################################### Executables to be compiled ####################################
add_executable(command_server src/CommandServer.cpp src/CommandInterface.cpp src/JointInterface.cpp src/iCubBase.cpp src/PositionControl.cpp src/Utilities.cpp src/QPSolver.cpp src/CartesianTrajectory.cpp src/Payload.cpp ${sources} ${headers})
target_link_libraries(command_server Eigen3::Eigen iDynTree::idyntree-high-level ${YARP_LIBRARIES})

add_executable(command_prompt src/CommandPrompt.cpp src/CommandInterface.cpp src/Utilities.cpp)
target_link_libraries(command_prompt Eigen3::Eigen ${YARP_LIBRARIES})
