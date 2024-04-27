add_library(UI SHARED
            src/mainwindow.cpp
            src/mainwindow.ui)
target_link_libraries(UI
                        ${REQUIRED_LIBS_QUALIFIED}
                        ${OpenCV_LIBS})


add_executable(test_ui_show
                include/mainwindow.h
                test/test_ui_show.cpp)
target_link_libraries(test_ui_show
                        UI
                        camera
                        ${catkin_LIBRARIES}
                        ${REQUIRED_LIBS_QUALIFIED})


