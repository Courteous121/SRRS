/**
 * @file Detection.h
 * @author 梁俊玮 (1609205169@qq.com)
 * @brief 车辆检测
 * @version 1.0
 * @date 2022-01-11
 * 
 * @copyright Copyright SCUT RobotLab(c) 2022
 * 
 */
#pragma once
#include "Robot.h"
#include "yolox.h"
#include "Sort.h"
#include <map>
#include <thread>
#include <algorithm>

// 裝甲板結構體
struct Armor
{
    int armorId = -1;   // 裝甲板id
    int trackerId = -1; // 來自哪一個跟蹤器
    float conf;         // 置信度
};

class Detector
{
public:
    Mat src;                   // 原图像
    Mat srcDraw;               // 画好的原图像
    vector<Point> point_map;   // 地图点
    vector<TrackBox> trackers; // 追蹤器

private:
    IExecutionContext *robotContext; // 车辆检测推理引擎
    IExecutionContext *armorContext; // 装甲板检测推理引擎
    bool baseFlag = false;

public:
    Detector(bool baseFlag = false);         // 构造函数
    ~Detector() {}                           // 析构函数
    void Detect(Mat &, vector<Robot_ptr> &); // 检测函數
    void Draw(vector<Robot_ptr> &);          // 显示函数

private:
    //检测前准备
    void InitEngine();                          // 初始化推理引擎
    void FindRobot(Mat &, vector<Robot_ptr> &); // 寻找机器人
};

using Detect_ptr = std::shared_ptr<Detector>;
