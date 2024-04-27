# 一帧图像里检测到每辆车信息的消息

uint8 carClass                      # 车辆的类别：10-14--红1~红5，1-5--蓝1~蓝5
uint32 carX                         # 车辆的中心点x轴图像坐标
uint32 carY                         # 车辆的中心点y轴图像坐标
uint32 carWidth                     # 车辆的图像宽度
uint32 carHeight                    # 车辆的图像高度
float64 carWorldX
float64 carWorldY
float64 carWorldZ
uint8 armorNum                      # 车辆图片里装甲板的个数
ArmorsInformation[] armorsInfo      # 车辆图像内装甲板图像信息