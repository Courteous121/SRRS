/**
 * @file Lidar.h
 * @author 梁俊玮 (1609205169@qq.com)
 * @brief 激光雷达库头文件
 * @version 1.0
 * @date 2022-01-14
 * 
 * @copyright Copyright SCUT RobotLab(c) 2022
 * 
 */
#pragma once
#include <iostream>           // std
#include <fstream>            // 文件处理
#include <future>             // 用于关闭激光雷达线程
#include "iomanip"            // 小数点处理
#include "client.h"           // ouster客户端库
#include "lidar_scan.h"       // 激光雷达扫描头文件
#include "build.h"            // ouster带的文件
#include "types.h"            // 激光雷达数据类型
#include "point_viz.h"        // 点云可视化头文件
#include "lidar_scan_viz.h"   // 雷达扫描可视化头文件
#include <condition_variable> // 条件变量
#include <thread>             // 多线程
#include <opencv2/opencv.hpp>
#include "Parameter.h"

using namespace std;
using namespace ouster;

// 激光雷達類
class Lidar
{
public:
    int connectStatus = 0; // 0代表未连接、1代表连接成功、-1代表连接失败
    cv::Mat depthMat;     // 深度图

private:
    bool getDepthFlag = false;                 // 获取深度图标志位
    size_t w;                                    // 每帧列数
    size_t h;                                    // 每列像素数
    int columnWindowLength;                    // 轨道窗口长度
    int lidarPort;                              // 激光雷达数据端口
    int imuPort;                                // 加速度计数据端口
    cv::Size imgSize;                           // 图像尺寸
    string sensorHostname;                      // 传感器地址
    string dataDestination;                     // 传感器要传输到的终点
    string prodLine;                            // 激光雷达多少线
    XYZLut lut;                                  // 查表法所需
    sensor::sensor_info info;                    // 激光雷达相关信息
    std::shared_ptr<sensor::client> handle;      // 权柄
    std::shared_ptr<ouster::LidarScan> lsRead;  // 后承接容器
    std::shared_ptr<ouster::LidarScan> lsWrite; // 前承接容器
    //扫描所使用的格式
    std::shared_ptr<uint8_t[]> lidarBuf;
    std::shared_ptr<ouster::ScanBatcher> batch;

private:
    void connect();   // 连接
    bool configure(); // 配置激光雷达

public:
    Lidar(string sensorHostname, string dataDestination, cv::Size imgSize = cv::Size(1920, 1080)); // 激光雷达构造函数
    void CalCoord(cv::Matx44d &CEM, cv::Matx33d &CM);                                                 // 计算深度图
    void OpenScan(std::future<void> &threadFuture);                                                  // 开启轮询扫描
    inline void ClearDepth() { this->depthMat = cv::Mat::zeros(imgSize, CV_64FC1); }                // 清空深度图
};

using Lidar_ptr = unique_ptr<Lidar>;