/**
 * @file Calibrator.h
 * @author 梁俊玮 (1609205169@qq.com)
 * @brief 标定工具类
 * @version 1.0
 * @date 2022-01-12
 * 
 * @copyright Copyright SCUT RobotLab(c) 2022
 * 
 */
#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Parameter.h"

#define CV_EVENT_MOUSEMOVE 0     //滑动
#define CV_EVENT_LBUTTONDOWN 1   //左键点击
#define CV_EVENT_RBUTTONDOWN 2   //右键点击
#define CV_EVENT_MBUTTONDOWN 3   //中键点击
#define CV_EVENT_LBUTTONUP 4     //左键放开
#define CV_EVENT_RBUTTONUP 5     //右键放开
#define CV_EVENT_MBUTTONUP 6     //中键放开
#define CV_EVENT_LBUTTONDBLCLK 7 //左键双击
#define CV_EVENT_RBUTTONDBLCLK 8 //右键双击
#define CV_EVENT_MBUTTONDBLCLK 9 //中键双击

using namespace std;
using namespace cv;

// 图像点集
static vector<Point2f> allImagePoint;
// 物体点集
static vector<Point2f> allObjectPoint;
// 标定器
class Calibrator
{
private:
    int pointClassNum; // 點的組數
public:
    Mat src;     // 标定原图
    Mat map;     // 标定用的地图
    Mat drawSrc; // 绘画了点后的标定图
    Mat drawMap; // 绘画了点后的地图
private:
    //PNP解算
    void PnpMap(vector<Point2f> &imagePoint, vector<Point2f> &objectPoint, Matx44d &pnpMatrixs);
    //透视变换
    void MapPrepare(Mat &img, vector<vector<Point2f>> &imagePoint, vector<vector<Point2f>> &objectPoint, vector<Mat> &warpMatrixs);


public:
    Calibrator();  //构造函数
    ~Calibrator(); //析构函数
    // 透视变换标定
    bool MapCalibration(const string &path, shared_ptr<VideoCapture> mainCap, shared_ptr<VideoCapture> baseCap);
    // 透视变换标定核心
    void MapCalibrationCore(const string &path, vector<Point2f> &allImagePoint, vector<Point2f> &allObjectPoint);
    // 可视化标记点
    void ShowProcess(vector<Point2f> &allImagePoint, vector<Point2f> &allObjectPoint);
};