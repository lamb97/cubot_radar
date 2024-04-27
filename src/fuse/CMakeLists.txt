add_library(fuse SHARED
            src/camera_radar_matrix.cpp
            src/camera_radar_matrix_param.cpp
            src/coordinate_transformer.cpp
            src/outparam.cpp)
target_link_libraries(fuse
                        camera
                        ${PCL_LIBRARIES}
                        ${OpenCV_LIBS})



add_executable(multi_sensor_subscriber
               test/multi_sensor_subscriber.cpp
        src/camera_radar_matrix.cpp
        src/camera_radar_matrix_param.cpp
        src/coordinate_transformer.cpp
        src/outparam.cpp)
target_link_libraries(multi_sensor_subscriber
                        common
                        camera
                        ${PCL_LIBRARIES}
                        ${OpenCV_LIBS}
                        radar
                        ${catkin_LIBRARIES})

add_executable(locate_target
                test/locate_target.cpp
        src/camera_radar_matrix.cpp
        src/camera_radar_matrix_param.cpp
        src/coordinate_transformer.cpp
        src/outparam.cpp)
target_link_libraries(locate_target
                        camera
                        ${PCL_LIBRARIES}
                        ${OpenCV_LIBS}
                        radar
                        ${catkin_LIBRARIES})

add_executable(fuse_info_subscriber
                test/fuse_info_subscriber.cpp
        src/camera_radar_matrix.cpp
        src/camera_radar_matrix_param.cpp
        src/coordinate_transformer.cpp
        src/outparam.cpp)
target_link_libraries(fuse_info_subscriber
                    camera
                    ${PCL_LIBRARIES}
                    ${OpenCV_LIBS}
                    uart
                    radar
                    ${tf2_LIBRARIES}
                    ${tf2_ros_LIBRARIES}
                    ${catkin_LIBRARIES})

add_executable(test_outparam
        test/test_outparam.cpp
        src/camera_radar_matrix.cpp
        src/camera_radar_matrix_param.cpp
        src/coordinate_transformer.cpp
        src/outparam.cpp)
target_link_libraries(test_outparam
                    camera
                    ${PCL_LIBRARIES}
                    ${OpenCV_LIBS}
                    ${catkin_LIBRARIES})


add_executable(fuse_info2
              test/fuse_info2.cpp
        src/camera_radar_matrix.cpp
        src/camera_radar_matrix_param.cpp
        src/coordinate_transformer.cpp
        src/outparam.cpp)
target_link_libraries(fuse_info2
                        camera
                        ${PCL_LIBRARIES}
                        ${OpenCV_LIBS}
                        uart
                        radar
                        ${tf2_LIBRARIES}
                        ${tf2_ros_LIBRARIES}
                        ${catkin_LIBRARIES})

add_executable(fuse2to1
               test/fuse2to1.cpp
        src/camera_radar_matrix.cpp
        src/camera_radar_matrix_param.cpp
        src/coordinate_transformer.cpp
        src/outparam.cpp)
target_link_libraries(fuse2to1
                        camera
                        ${PCL_LIBRARIES}
                        ${OpenCV_LIBS}
                        uart
                        ${catkin_LIBRARIES})