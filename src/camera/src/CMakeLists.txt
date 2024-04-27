add_library(camera SHARED
            include/camera/huaray_camera.h
            huaray_camera.cpp
            huaray_camera_data.cpp
            huaray_camera_hardware_param.cpp
            huaray_camera_model_param.cpp
            huaray_camera_param.cpp
            huaray_camera_runtime_param.cpp
            huaray_camera_status.cpp 
            )

target_link_libraries(camera
                      ${HUARAY_LIBS}
                      ${OpenCV_LIBS}
                      common
                      logger)
