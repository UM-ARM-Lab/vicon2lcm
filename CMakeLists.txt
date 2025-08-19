cmake_minimum_required(VERSION 3.8)
project(lcm_vicon_bridge)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add user's local lib to CMAKE_PREFIX_PATH for LCM
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/.local/lib/python3.10/site-packages/lib/lcm/cmake")

# Find required packages
find_package(lcm REQUIRED)
find_package(PkgConfig REQUIRED)

# Vicon SDK paths
if(CMAKE_SIZEOF_VOID_P MATCHES "8")
    set(VICON_SDK_PATH vicon_sdk/x64)
    set(VICON_SDK_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/${VICON_SDK_PATH})
    set(VICON_SDK_LINK_DIR ${PROJECT_SOURCE_DIR}/${VICON_SDK_PATH})
else()
    set(VICON_SDK_PATH vicon_sdk/x86)
    set(VICON_SDK_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/${VICON_SDK_PATH})
    set(VICON_SDK_LINK_DIR ${PROJECT_SOURCE_DIR}/${VICON_SDK_PATH})
endif()

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${VICON_SDK_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/vicon_msgs
)

# Check if mbot_lcm_base is available
if(EXISTS "${PROJECT_SOURCE_DIR}/../mbot_lcm_base/mbot_msgs/lcmtypes/mbot_lcm_msgs/pose3D_t.hpp")
    include_directories(${PROJECT_SOURCE_DIR}/../mbot_lcm_base/mbot_msgs/lcmtypes)
    add_definitions(-DMBOT_LCM_AVAILABLE)
    message(STATUS "mbot_lcm_base found - pose3D_t support enabled")
else()
    message(STATUS "mbot_lcm_base not found - using raw binary messages")
endif()

# Link directories
link_directories(${VICON_SDK_LINK_DIR})

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# LCM Vicon Bridge executable
add_executable(lcm_vicon_bridge src/lcm_vicon_bridge.cpp)

# Vicon State Receiver executable
add_executable(vicon_state_receiver src/vicon_state_receiver.cpp)

# Link libraries
target_link_libraries(lcm_vicon_bridge
    lcm
    ViconDataStreamSDK_CPP
)

target_link_libraries(vicon_state_receiver
    lcm
)

# Set RPATH for Vicon SDK
set_target_properties(lcm_vicon_bridge PROPERTIES 
    INSTALL_RPATH "$ORIGIN;${VICON_SDK_LINK_DIR}"
)

# Install targets
install(TARGETS lcm_vicon_bridge vicon_state_receiver
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if they exist
if(EXISTS "${PROJECT_SOURCE_DIR}/launch")
    install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
    )
endif()

# Install config files if they exist
if(EXISTS "${PROJECT_SOURCE_DIR}/config")
    install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}/
    )
endif()

# Testing
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
endif()
