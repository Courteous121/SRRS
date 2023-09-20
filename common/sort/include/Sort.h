/**
 * @file Sort.h
 * @author 梁俊玮 (1609205169@qq.com)
 * @brief 目标追踪类
 * @version 1.0
 * @date 2021-11-27
 * 
 * @copyright Copyright SCUT RobotLab(c) 2022
 * 
 */
#pragma once
#include <iostream>
#include <fstream>
#include <iomanip>
#include <unistd.h>
#include <set>
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "Parameter.h"
#include "Hungarian.h"
#include "KalmanTracker.h"

using namespace std;
using namespace cv;

// 目標檢測盒子
struct DetectBox
{
    int id;            // 機器人編號
    float unblockProb; // 未被遮擋的概率，即機器人的置信度
    Rect_<float> box;  // 機器人在圖中的位置
};

//卡尔曼滤波盒子
struct TrackBox
{
    bool resetFlag = true;   // 重新分類標誌位
    bool outputFlag = false; // 輸出標誌位
    int id = -1;             // 追踪目标的车辆id
    int resetRank = 0;       // 重新分類等級
    int resetCount = 0;      // 重置计时器
    int classfiCount = 0;    // 连续分类计时器
    int situation = -1;      // 记录输出情况（调试用）-1代表刚创建，1代表进行了有检测更新，2代表已经进行了预测
    float armorProb = 0;     // 追踪目标的概率
    float unblockProb = 0;   // 没被遮挡的概率
    KalmanTracker tracker;   // 卡尔曼追踪器
};

//计算iou
double GetIOU(Rect_<float> box_a, Rect_<float> box_b);
//目标追踪
int TargetSort(vector<DetectBox> &, vector<TrackBox> &);