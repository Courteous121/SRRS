/**
 * @file Robot.h
 * @author 梁俊瑋 (1609205169@qq.com)
 * @brief 機器人類
 * @version 0.1
 * @date 2023-04-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

static const char *class_names[] = {
    "blue1",
    "blue2",
    "blue3",
    "blue4",
    "blue5",
    "red1",
    "red2",
    "red3",
    "red4",
    "red5",
    "grey1",
    "grey2",
    "grey3",
    "grey4",
    "grey5"};

// 機器人類
class Robot
{
private:
    bool findFlag = false;   // 機器人是否被找到
    bool baseFlag = true;    // 機器人是否來自基地相機
    int robotId = -1;        // 机器人的编号,-1为无法确定其装甲板编号
    int blood = -1;          // 机器人的血量,-1为无法确定
    float armorProb = 0.0;   // 其装甲板编号的准确性
    float unblockProb = 0.0; // 其没被遮挡的概率
    Point mapLocation;       // 机器人在地图上的位置
    Rect imgLocation;        // 机器人在图像中的位置

public:
    // 构造机器人
    Robot() {}
    // 获取机器人的编号
    inline int getID() { return robotId; }
    // 获取装甲板编号的准确性
    inline float getArmorProb() { return armorProb; }
    // 获取没被遮挡的概率
    inline float getUnBlockProb() { return unblockProb; }
    // 获取机器人的血量
    inline int getBlood() { return blood; }
    // 获取机器人的是否被找到
    inline bool getFindFlag() { return findFlag; }
    // 获取机器人的基地相機標誌位
    inline bool getBaseFlag() { return baseFlag; }
    // 获取机器人在图像中的位置
    inline Rect getImgLoaction() { return imgLocation; }
    // 获取机器人在地图上的位置
    inline Point getMapLocation() { return mapLocation; }
    // 設置機器人ID
    inline void setRobotId(int id) { this->robotId = id; }
    // 設置機器人的裝甲板置信度
    inline void setArmorConf(float conf) { this->armorProb = conf; }
    // 設置機器人的没被遮挡的概率
    inline void setUnBlockProb(float unblockProb) { this->unblockProb = unblockProb; }
    // 設置機器人是否被找到
    inline void setFindFlag(bool findFlag) { this->findFlag = findFlag; }
    // 设置机器人在地图上的位置
    inline void setMapLocation(Point mapResult) { mapLocation = mapResult; }
    // 设置机器人在圖像上的位置
    inline void setImgLocation(Rect imgResult) { imgLocation = imgResult; }
    // 設置機器人的基地相機標誌位
    inline void setBaseFlag(bool baseFlag) { this->baseFlag = baseFlag; }
    // 設置基礎信息
    inline void setBaseInformation(float armorProb, float unblockProb, Point mapLocation, Rect imgLocation)
    {
        this->armorProb = armorProb;
        this->unblockProb = unblockProb;
        this->mapLocation = mapLocation;
        this->imgLocation = imgLocation;
    }
};

using Robot_ptr = std::shared_ptr<Robot>;