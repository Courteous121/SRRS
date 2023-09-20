/**
 * @file RadarControl.h
 * @author 梁俊玮 (1609205169@qq.com)
 * @brief 雷达站伺服控制器头文件
 * @version 1.0
 * @date 2022-01-12
 * 
 * @copyright Copyright SCUT RobotLab(c) 2022
 * 
 */
#pragma once
#include "Locator.h"
#include "Detector.h"
#include "RMVideoCapture.h"
#include "refereeio.h"
#include "MatIO.h"
#include "Lidar.h"
#include <thread>

//雷达站类
class Radar
{
public:
    Mat frame;        // 每帧图像
    Mat baseFrame;    // 基地圖像
    Mat dartFrame;    // dart圖像
    Mat depthMat;     // 深度图
    Mat depthMatDraw; // 画好的深度图

private:
    bool judgeFlag = false;                        // 是否连接了裁判系统
    bool calibrationFlag;                          // 标定标志位
    int changeFlag = 0;                            // 内置时钟被改变标志位
    int frameIndex = 0;                            // 帧数
    double getStartTime = -1;                      // 得到裁判系统时间的开始时间
    string path;                                   // 视频路径
    string hostname;                               // 激光雷达主机地址
    string matName = path_param.depthmatsave_path; // 深度图存储地址
    Detect_ptr mainDetector = nullptr;             // 主检测器指针
    Detect_ptr baseDetector = nullptr;             // 基地檢測器指針
    Locate_ptr locator = nullptr;                  // 定位器指針
    Lidar_ptr lidar = nullptr;                     // 激光雷达指针
    vector<Robot_ptr> robots;                      // 管理所有机器人的向量
    shared_ptr<VideoCapture> mainCap = nullptr;    // 主相机指针
    shared_ptr<VideoCapture> baseCap = nullptr;    // 基地监控相机
    shared_ptr<VideoCapture> dartCap = nullptr;    // 飞镖监控相机
    shared_ptr<refereeio> judge;                   // 裁判系统
    mutex clockMutex;                              // 内置时钟锁
    thread poll;                                   // 激光雷达扫描线程
    promise<void> pollExit;                        // 激光雷达扫描线程退出信号
    future<void> pollFuture;                       // 与promise关联的future对象
    thread judgeThread;                            // 消息队列线程
    promise<void> judgeExit;                       // 消息队列线程退出信号
    future<void> judgeFuture;                      // 与promise关联的future对象
    thread clockThread;                            // 内部时钟线程
    promise<void> clockExit;                       // 内部时钟线程退出信号
    future<void> clockFuture;                      // 与promise关联的future对象
    mutex mainCapMutex;                            // 主相机读取锁
    thread mainCapThread;                          // 读取主相机线程
    promise<void> mainCapExit;                     // 读取主相机线程退出信号
    future<void> mainCapFuture;                    // 与promise关联的future对象
    mutex baseCapMutex;                            // 基地相机读取锁
    thread baseCapThread;                          // 读取基地相机线程
    promise<void> baseCapExit;                     // 读取基地相机线程退出信号
    future<void> baseCapFuture;                    // 与promise关联的future对象
    Mat mainWait;                                  // 主模组线程缓冲图像
    Mat baseWait;                                  // 主模组线程缓冲图像

public:
    Radar(string filename, string hostname, bool calibrationFlag = false); // 构造函数
    ~Radar();                                                              // 析构函数
    bool Run();                                                            // 处理函数
    void ReadBase(Mat &baseFrame);                                         // 读取主相机的函数
    bool getBaseFlag() { return this->baseCap != nullptr; }
    bool getDartFlag() { return this->dartCap != nullptr; }
    const Mat &getDart() { return this->dartFrame; }
    const Mat &getSrcDraw() { return this->mainDetector->srcDraw; }
    const Mat &getBaseDraw() { return this->baseDetector->srcDraw; }
    const Mat &getMapDraw() { return this->locator->getMapDraw(); }
    string getPath() { return this->path; }
    shared_ptr<VideoCapture> getMainCap() { return this->mainCap; }
    shared_ptr<VideoCapture> getBaseCap() { return this->baseCap; }

private:
    float fps(float);                               // 计算帧率
    void InitCamera();                              // 初始化相机
    void RunMain(future<void> &threadFuture);       // 运行主要相机线程函数
    void RunBase(future<void> &threadFuture);       // 运行基地相机线程函数
    void ReceiveJudge(future<void> &threadFuture);  // 接收裁判系统数据线程函数
    void InternalClock(future<void> &threadFuture); // 内部时钟线程函数
};

using Radar_ptr = unique_ptr<Radar>;