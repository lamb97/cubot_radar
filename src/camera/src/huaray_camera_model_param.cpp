//
// Created by plutoli on 2022/4/5.
//

#include "huaray_camera_model_param.h"

// ******************************  HuarayCameraModelParam类的公有函数  ******************************

// 构造函数
HuarayCameraModelParam::HuarayCameraModelParam():
        CvInternalMatrix(),
        CvExternalMatrix(),
        CvDistortionVector(),
        EigenInternalMatrix(),
        EigenExternalMatrix(),
        EigenDistortionVector()
{
}