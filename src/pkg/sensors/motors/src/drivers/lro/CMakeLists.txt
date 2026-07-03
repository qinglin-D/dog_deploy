AUX_SOURCE_DIRECTORY(. SOURCE_LIST_LRO_MOTORS)
add_library(lro_motors
        STATIC
        ${SOURCE_LIST_LRO_MOTORS}
        lro_motor_driver_can.cpp

)
target_include_directories(lro_motors PUBLIC ./ ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(lro_motors PUBLIC ${PUBLIC_DEPENDENCIES} motors_canfd)
