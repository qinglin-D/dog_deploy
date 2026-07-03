cmake_minimum_required(VERSION 3.14)
project(imu LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME}  STATIC 
    src/crc16.cpp
    src/serial_port.cpp
    src/imu_parser.cpp
)

target_include_directories(${PROJECT_NAME} 
    PRIVATE 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_compile_options(${PROJECT_NAME}  PRIVATE -Wall -Wextra -Wpedantic)


install(DIRECTORY src/
    INCLUDES DESTINATION include/
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)


install(TARGETS ${PROJECT_NAME} 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

if(ament_cmake_FOUND)
    ament_export_libraries(${PROJECT_NAME})
    ament_export_include_directories(include)
    ament_package()
endif()
