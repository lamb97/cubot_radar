add_library(uart SHARED
            src/uart.cpp
            src/serial_port.cpp
            src/serial_port_param.cpp
            src/robot_command.cpp)
target_link_libraries(uart
                        ${TORCH_LIBRARIES}
                        ${OpenCV_LIBS})



find_package(catkin REQUIRED COMPONENTS
            roscpp
            rospy
            std_msgs
            geometry_msgs
            sensor_msgs
            message_generation
            message_filters
            rosbag
            pcl_ros
            image_transport
            cv_bridge)

add_executable(test_uart
        test/test_uart.cpp)
target_link_libraries(test_uart
        uart)


add_executable(test_serial_port
        test/test_serial_port.cpp
        test/test_serial.cpp)
target_link_libraries(test_serial_port
        uart)

add_executable( test_radar_uart
        test/test_radar_uart.cpp)
target_link_libraries(test_radar_uart
        uart
        fuse
        ${catkin_LIBRARIES})

add_executable(test_serial_port_param
        test/test_serial_port_param.cpp)
target_link_libraries(test_serial_port_param
        uart)