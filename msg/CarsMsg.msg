# YOLOv5网络网络检测一帧图像得到的车辆信息

Header header                   # ROS标准消息头部，使发布消息更利于观测和分析
uint64 timebase                 # 检测出来第一帧车辆信息的时间戳
uint8 carNum                    # 图像里车辆个数
CarsInformation[] carsInfo      # 图像里的车辆信息
sensor_msgs/Image image         # 图像信息