//
// Created by plutoli on 2021/8/9.
//

#ifndef CUBOT_BRAIN_DELTA_CV_H
#define CUBOT_BRAIN_DELTA_CV_H

#include <cassert>
#include <opencv2/opencv.hpp>
#include <mmintrin.h>       // MMX
#include <xmmintrin.h>      // SSE
#include <emmintrin.h>      // SSE2
#include <immintrin.h>      // AVX
#include <pmmintrin.h>      // SSE3
#include <smmintrin.h>
#include "bgr_weight.h"
#include "hsv_threshold.h"

/**
 * @brief (a >=b)? 0xFF: 0x00
 */
#define _mm_cmpge_epu8(a, b) _mm_cmpeq_epi8(_mm_max_epu8(a, b), a)

/**
 * @brief (a <=b)? 0xFF: 0x00
 */
#define _mm_cmple_epu8(a, b) _mm_cmpeq_epi8(_mm_min_epu8(a, b), a)

/**
 * @brief (a >=b)? 0xFF: 0x00
 */
#define _mm256_cmpge_epu8(a, b) _mm256_cmpeq_epi8(_mm256_max_epu8(a, b), a)

/**
 * @brief (a <=b)? 0xFF: 0x00
 */
#define _mm256_cmple_epu8(a, b) _mm256_cmpeq_epi8(_mm256_min_epu8(a, b), a)

/**
 * @brief 将两个__mm128i 拼接成一个__mm256i a为低地址，b为高地址
 */
#define _mm256_combine_si128(a, b) _mm256_insertf128_si256(_mm256_castsi128_si256(a), b, 1)

/**
 * @brief 以王海波的图像预处理加速程序为基础整理的视觉算法库
 * https://software.intel.com/sites/landingpage/IntrinsicsGuide/#
 */
class DeltaCV
{
public:

    /**
     * @brief 加权二值化
     * @param[in] rawImage          原始图像
     * @param[in] weight            原始图像的各通道权值
     * @param[in] threshold         原始图像各通道加权后的分割阈值
     * @param[out] grayScaleImage   分割后得到的灰度图像
     * @note 原始图像的各通道权值之和必须等于1
     */
    static void WeightedBinarize(const cv::Mat &rawImage,
                                 const BGRWeight &weight,
                                 const unsigned char &threshold,
                                 cv::Mat *grayScaleImage);

    /**
     * @brief 快速加权二值化
     * @param[in] rawImage          原始图像
     * @param[in] weight            原始图像的各通道权值
     * @param[in] threshold         原始图像各通道加权后的分割阈值
     * @param[out] grayScaleImage   分割后得到的灰度图像
     * @note 原始图像的各通道权值之和必须等于1
     */
    static void FastWeidhtedBinarize(const cv::Mat &rawImage,
                                     const BGRWeight &weight,
                                     const unsigned char &threshold,
                                     cv::Mat *grayScaleImage);

    /**
     * @brief HSV三通道阈值分割
     * @param[in] rawImage          原始图像
     * @param[in] threshold         各通道阈值上下限
     * @param[out] grayScaleImage   分割后得到的灰度图像
     */
    static void InRange(const cv::Mat &rawImage,
                        const HSVThreshold &threshold,
                        cv::Mat *grayScaleImage);

    /**
     * @brief 快速HSV三通道阈值分割
     * @param[in] rawImage          原始图像
     * @param[in] threshold         各通道阈值上下限
     * @param[out] grayScaleImage   分割后得到的灰度图像
     */
    static void FastInRange(const cv::Mat &rawImage,
                            const HSVThreshold &threshold,
                            cv::Mat *grayScaleImage);

    /**
     * @brief  获得彩色图像中像素点的像素值
     * @param[in] rawImage          原始图像
     * @param[out] pixelValue       载入像素值的容器
     */
    static void getPixelValue(const cv::Mat &rawImage,
                              std::vector<std::vector<int>> *pixelValue);
};

#endif //CUBOT_BRAIN_DELTA_CV_H