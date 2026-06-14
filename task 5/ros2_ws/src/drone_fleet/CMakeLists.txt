cmake_minimum_required(VERSION 3.16)
project(drone_fleet CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# ── Find dependencies ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(std_msgs    REQUIRED)
find_package(std_srvs    REQUIRED)

# ── Shared drone-logic library (the OOP classes, header-only interface) ────────
set(DRONE_SOURCES
    src/Vehicle.cpp
    src/Drone.cpp
    src/MissionDrone.cpp
    src/AutonomousDrone.cpp
)

add_library(drone_lib STATIC ${DRONE_SOURCES})
target_include_directories(drone_lib PUBLIC include include/drone_fleet)

# ── drone_node ────────────────────────────────────────────────────────────────
add_executable(drone_node src/drone_node.cpp)
target_include_directories(drone_node PRIVATE include)
target_link_libraries(drone_node drone_lib)
ament_target_dependencies(drone_node rclcpp std_msgs std_srvs)

# ── fleet_manager ─────────────────────────────────────────────────────────────
add_executable(fleet_manager src/fleet_manager.cpp)
target_include_directories(fleet_manager PRIVATE include)
target_link_libraries(fleet_manager drone_lib)
ament_target_dependencies(fleet_manager rclcpp std_msgs std_srvs)

# ── health_monitor ────────────────────────────────────────────────────────────
add_executable(health_monitor src/health_monitor.cpp)
target_include_directories(health_monitor PRIVATE include)
ament_target_dependencies(health_monitor rclcpp std_msgs std_srvs)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS drone_node fleet_manager health_monitor
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
