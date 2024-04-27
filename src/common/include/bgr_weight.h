//
// Created by plutoli on 2021/8/12.
//

#ifndef CUBOT_BRAIN_BGR_WEIGHT_H
#define CUBOT_BRAIN_BGR_WEIGHT_H

/**
 * @brief 图像的BGR三通道权值
 */
class BGRWeight
{
public:
    float Blue;     ///< 蓝色通道权值
    float Green;    ///< 绿色通道权值
    float Red;      ///< 红色通道权值

    /**
     * @brief 构造函数
     */
    BGRWeight();

    /**
     * @brief 析构函数
     */
    ~BGRWeight() = default;

    /**
     * @brief 判断BGR三通道权值是否有效
     * @return 权值是否有效\n
     *         -<em>false</em> 权值无效\n
     *         -<em>true</em> 权值有效\n
     */
    bool IsValid() const;
};

#endif //CUBOT_BRAIN_BGR_WEIGHT_H