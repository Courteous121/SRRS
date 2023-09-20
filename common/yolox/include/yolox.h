/**
 * @file DeepLearning.h
 * @author 梁俊玮 (1609205169@qq.com)
 * @brief 深度学习类
 * @version 1.0
 * @date 2021-11-27
 * 
 * @copyright Copyright SCUT RobotLab(c) 2022
 * 
 */
#pragma once
#include <fstream>
#include <iostream>
#include <sstream>
#include <numeric>
#include <chrono>
#include <vector>
#include <opencv2/opencv.hpp>
#include <dirent.h>
#include "NvInfer.h"
#include "cuda_runtime_api.h"
#include "logging.h"
#include "Parameter.h"

using namespace nvinfer1;

// 目標檢測結果
struct Object
{
    cv::Rect_<float> rect;
    int label;
    float prob;
};

//反序列化模型
IExecutionContext *DeserializeEngine(string, int);
//检测目標
void DetectTarget(Mat &, IExecutionContext *, vector<Object> &, int);