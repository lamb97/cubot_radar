add_library(camera SHARED
            include/huaray_camera.h
            src/huaray_camera.cpp
            src/huaray_camera_data.cpp
            src/huaray_camera_hardware_param.cpp
            src/huaray_camera_model_param.cpp
            src/huaray_camera_param.cpp
            src/huaray_camera_runtime_param.cpp
            src/huaray_camera_status.cpp)

target_link_libraries(camera
                      ${HUARAY_LIBS}
                      ${OpenCV_LIBS}
                      common
                      logger)


add_executable(test_huaray_camera
               test/test_huaray_camera.cpp)
target_link_libraries(test_huaray_camera
                      camera)

add_executable(test_huaray_camera_data
               test/test_huaray_camera_data.cpp)
target_link_libraries(test_huaray_camera_data
                      camera)

add_executable(test_huaray_camera_param
               test/test_huaray_camera_param.cpp)
target_link_libraries(test_huaray_camera_param
                      camera)

add_executable(test_huaray_camera_status
               test/test_huaray_camera_status.cpp)
target_link_libraries(test_huaray_camera_status
                      camera)

add_executable(test_image_publisher
               test/test_image_publisher.cpp)
target_link_libraries(test_image_publisher
                      camera
                      ${catkin_LIBRARIES})
