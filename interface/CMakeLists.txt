####################################################################################################
#                                                                                                  #
#                    Generate YARP Thrift interface for bimanual controller                        #
#                                                                                                  #
####################################################################################################

cmake_minimum_required(VERSION 3.12)

project(manipulation_interface LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Create the component (navigation client)
find_package(YARP COMPONENTS os dev idl_tools REQUIRED)

add_library(manipulation_interface STATIC)
# Create the grasp_interface C++ Library
yarp_add_idl(IDL_GEN_FILES manipulation_interface.thrift)

target_sources(manipulation_interface
    PRIVATE
    ${IDL_GEN_FILES}
)
