/**
 * @file Locator.h
 * @author 梁俊瑋 (1609205169@qq.com)
 * @brief 定位器
 * @version 0.1
 * @date 2023-04-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include "Lidar.h"
#include "Robot.h"
#include <iostream>

using namespace std;

class Locator
{
private:
    bool damageFlag = false;    // 損壞標誌位
    double clusterNum[3] = {0}; // 聚類點數
    double clusterAll[3] = {0}; // 聚類總值
    Matx44d pnpMatrixs;         // pnp计算出来的旋转矩阵
    Mat map;                    // 地图
    Mat mapDraw;                // 地圖繪製版
    Mat depthMat;               // 深度图
    Mat calibrationMap;         // 标定用的地图
    Mat depthMatDraw;           // 用于调试的画框框深度图
    Mat warpMatrix;             // 基地透视变换矩阵
    vector<int> mapId;          // 映射到地圖上的編號
    vector<Point> mapPoint;     // 映射到地圖上的點
    vector<Mat> warpMatrixs;    // 透视变换矩阵序列

private:
    double KRepresent(Rect &imgLocation);                                        // k聚类獲得深度代表值的方法
    void KMeans(int edge[4], double clusters[3]);                                // k聚類內核
    void DoMap(vector<Mat> &warpMatrixs, Point &pointDect, Point &result);       // 透视变换
    void PeTrans(Mat &warpMatrix, double &xmap, double &ymap, Point &dectPoint); // 透視變換執行

public:
    Locator();                                   // 構造函數
    void LocateRobot(vector<Robot_ptr> &robots); // 定位机器人
    void Draw(vector<Robot_ptr> &robots);        // 繪製函數
    inline Mat getDepthMatDraw() { return this->depthMatDraw; }
    const Mat &getMapDraw() { return this->mapDraw; }
    inline vector<int> &getMapId() { return this->mapId; }
    inline vector<Point> &getMapPoint() { return this->mapPoint; }
    inline void setDepthMat(Mat &depthMat) { this->depthMat = depthMat; }
    inline void setDepthMatDraw(Mat &depthMatDraw) { this->depthMatDraw = depthMatDraw; }
};

using Locate_ptr = std::shared_ptr<Locator>;