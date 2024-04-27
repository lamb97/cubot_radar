add_library(radar SHARED
            src/point_cloud_filter.cpp
            src/livox_radar_subscriber.cpp)
target_link_libraries(radar
                        fuse
                        ${PCL_LIBRARIES}
                        ${catkin_LIBRARIES})


add_executable(test_radar_subscriber
                  test/test_radar_subscriber.cpp)
target_link_libraries(test_radar_subscriber
                        common
                        fuse
                        ${catkin_LIBRARIES})

add_executable(test_radar_publisher
                test/test_radar_publisher.cpp)
target_link_libraries(test_radar_publisher
                        radar)

add_executable(test_pointcloud_coloring
               test/test_pointcloud_coloring.cpp)
target_link_libraries(test_pointcloud_coloring
                        fuse
                        common)

add_executable(test_radar_depth_map
                test/test_radar_depth_map.cpp)
target_link_libraries(test_radar_depth_map
                        fuse)

add_executable(test_radar_gathering
                test/test_radar_gathering.cpp)
target_link_libraries(test_radar_gathering
                        fuse
                        radar
                        common
                        ${catkin_LIBRARIES})