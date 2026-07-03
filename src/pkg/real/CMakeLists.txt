cmake_minimum_required(VERSION 3.18)
project(real)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
# set(fmt_DIR "src/3rdparty/fmt-12.1.0-h44f220b_0")
# find_package(fmt REQUIRED)
# find_package(spdlog REQUIRED)

find_package(motors REQUIRED)
find_package(imu REQUIRED)
find_package(gamepad REQUIRED)
find_package(rl_controller REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

###############
## # build  ###
###############
file(GLOB REAL_FILES "src/*.cpp")
add_executable(${PROJECT_NAME} ${REAL_FILES})

target_include_directories(${PROJECT_NAME} 
    PUBLIC
      include
      ${EIGEN3_INCLUDE_DIRS}
      ${rl_controller_INCLUDE_DIRS}
      ${gamepad_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME} 
    PUBLIC
    motors
    std_msgs
    sensor_msgs
    imu
    Eigen3
    rclcpp  
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC 
    # fmt::fmt spdlog::spdlog
    yaml-cpp
    ${gamepad_LIBRARIES}
)


###############
##  Install  ##
###############
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY include/${PROJECT_NAME}
    INCLUDES DESTINATION include/
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

target_compile_definitions(${PROJECT_NAME} 
    PRIVATE
        CONFIG_PATH="${CMAKE_SOURCE_DIR}/../../config"
        LOG_PATH="${CMAKE_SOURCE_DIR}/../../../"
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
