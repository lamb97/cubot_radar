include_directories(./include)
link_directories(./lib1)

add_library(detect SHARED
            src/detect_param.cpp
            src/detect_yolo.cpp
            src/detect_cars_info.cpp
            src/detect_armors_info.cpp)
target_link_libraries(detect
                      #${TORCH_LIBRARIES}
                      ${OpenCV_LIBS}
                      onnxruntime
                      common)



#[[
add_executable(test_import_model
               test/test_import_model.cpp)
target_link_libraries(test_import_model
                      detect)]]

add_executable(test_target_detect
               test/test_target_detect.cpp
        src/detect_param.cpp
        src/detect_yolo.cpp
        src/detect_cars_info.cpp
        src/detect_armors_info.cpp)
target_link_libraries(test_target_detect
                        ${OpenCV_LIBS}
                        onnxruntime
                        common
                      camera
                      ${catkin_LIBRARIES})

add_executable(detect_result_publisher
               test/detect_result_publisher.cpp
        src/detect_param.cpp
        src/detect_yolo.cpp
        src/detect_cars_info.cpp
        src/detect_armors_info.cpp)
target_link_libraries(detect_result_publisher
                        ${OpenCV_LIBS}
                        onnxruntime
                        common
                        camera
                        ${catkin_LIBRARIES})

add_executable(detect_result_publisher2
                test/detect_result_publisher2.cpp
        src/detect_param.cpp
        src/detect_yolo.cpp
        src/detect_cars_info.cpp
        src/detect_armors_info.cpp)
target_link_libraries(detect_result_publisher2
                        ${OpenCV_LIBS}
                        onnxruntime
                        common
                        camera
                        ${catkin_LIBRARIES})

#[[
add_executable(detect_result_subscriber
                test/detect_result_subscriber.cpp)
target_link_libraries(detect_result_subscriber
                        detect
                        ${catkin_LIBRARIES})]]
