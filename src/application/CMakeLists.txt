add_executable(get_pointcloud_coordinate
               get_pointcloud_coordinate.cpp)
target_link_libraries(get_pointcloud_coordinate
                      ${PCL_LIBRARIES}
                      ${Boost_LIBRARIES})

add_executable(get_pixel_coordinate
               get_pixel_coordinate.cpp)
target_link_libraries(get_pixel_coordinate
                      ${OpenCV_LIBS})

add_executable(camera_radar_coordinate_transform
               camera_radar_coordinate_transform.cpp)
target_link_libraries(camera_radar_coordinate_transform
                      fuse)