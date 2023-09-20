/**
 * @file RadarControl.cpp
 * @author 梁俊玮 (1609205169@qq.com)
 * @brief 雷达站伺服控制器
 * @version 1.0
 * @date 2022-01-12
 * 
 * @copyright Copyright SCUT RobotLab(c) 2022
 * 
 */

#include "Radar.h"

Radar::Radar(string filename, string hostname, bool calibrationFlag)
{
    // 賦值
    this->path = filename;                   // 初始化视频路径
    this->hostname = hostname;               // 激光雷达主机地址
    this->calibrationFlag = calibrationFlag; // 标定标志位
    gameinfo.game_side = 1;                  // 定义我们属于哪一方
    // 初始化足夠多的機器人
    for (int i = 0; i < 10; i++)
    {
        // 构造机器人对象
        Robot_ptr robot(new Robot());
        robot->setRobotId(i);
        // 将识别出的车 push 到 robots 中
        this->robots.emplace_back(robot);
    }

    // 初始化相机
    InitCamera();
    // 如果是标定，则读取相机即可退出
    if (calibrationFlag)
    {
        return;
    }
    // 主相机准备
    //开启线程
    if (this->path.empty())
    {
        this->mainCapFuture = this->mainCapExit.get_future();
        mainCapThread = thread([&]
                               { RunMain(mainCapFuture); });
    }
    // 副相机准备
    if (baseCap != nullptr && this->baseCap->isOpened())
    {
        this->baseCapFuture = this->baseCapExit.get_future();
        baseCapThread = thread([&]
                               { RunBase(baseCapFuture); });
    }
    mainDetector = Detect_ptr(new Detector()); // 初始化检测器
    if (baseCap != nullptr)
    {
        baseDetector = Detect_ptr(new Detector(true)); // 初始化检测器
    }
    locator = Locate_ptr(new Locator()); // 初始化定位器
    if (path_param.islidar)
    {
        lidar = Lidar_ptr(new Lidar(hostname, "", Size(1920, 1080))); // 初始化激光雷達
    }

    //准备线程future对象
    this->pollFuture = this->pollExit.get_future();
    this->judgeFuture = this->judgeExit.get_future();
    this->clockFuture = this->clockExit.get_future();

    // 準備裁判系統
    this->judge = shared_ptr<refereeio>(new refereeio);
    if (!hostname.empty() && path_param.islidar)
    {
        poll = thread([&]
                      { this->lidar->OpenScan(pollFuture); });
    }

    if (writeBlackBoard(this->judge))
    {
        this->judgeFlag = true;
        //开启线程
        judgeThread = thread([&]
                             { ReceiveJudge(judgeFuture); });
        clockThread = thread([&]
                             { InternalClock(clockFuture); });
    }
    else
    {
        gameinfo.game_side = 1; // 定义我们属于哪一方
    }
    // 初始化規定區域
    WriteArea();
}
/**
 * @brief 析构函数
 */
Radar::~Radar()
{
    // 释放资源
    // 释放资源
    if (this->calibrationFlag != true)
    {
        mainCapExit.set_value(); //终止线程
        mainCapThread.join();    //等待线程结束
    }
    mainCap->release();
    mainCap = nullptr;
    if (baseCap != nullptr && baseCap->isOpened() && this->calibrationFlag != true)
    {
        baseCap->release();
        baseCap = nullptr;
    }
    if (dartCap != nullptr && dartCap->isOpened() && this->calibrationFlag != true)
    {
        dartCap->release();
        dartCap = nullptr;
    }
    if ((!hostname.empty() || path_param.islidar) && this->calibrationFlag != true)
    {
        pollExit.set_value(); //终止线程
        poll.join();          //等待线程结束
    }
    if (calibrationFlag)
    {
        //如果是标定，则读取相机即可退出
        return;
    }
    if (judgeFlag)
    {
        judgeExit.set_value(); //终止线程
        judgeThread.join();    //等待线程结束
        clockExit.set_value(); //终止线程
        clockThread.join();    //等待线程结束
    }
}

/**
 * @brief 处理函数
 * 
 * @return 图像是否加载成功
 */
bool Radar::Run()
{
    double ts = (double)cv::getTickCount();
    bool depthFlag = true;
    //读取图片
    thread readPoll = thread([&]
                             {
                                 if (this->path.empty())
                                 {
                                     mainCapMutex.lock();
                                     frame = mainWait.clone();
                                     mainCapMutex.unlock();
                                 }
                                 else
                                 {
                                     mainCap->read(frame);
                                 }
                             });
    thread basePoll = thread([&]
                             {
                                 if (baseCap != nullptr)
                                 {
                                     baseCapMutex.lock();
                                     baseFrame = baseWait.clone();
                                     baseCapMutex.unlock();
                                 }
                             });
    thread dartPoll =thread([&]
                             {
                                 if (dartCap != nullptr)
                                 {
                                     dartCap->read(dartFrame);
                                 }
                             });

    //获取深度图
    thread depthPoll = thread([&]
                              {
                                  if (this->path.empty()) //如果没有视频路径（用摄像头）
                                  {
                                      if (path_param.islidar)
                                      {
                                          //先清空之前的深度图
                                          this->lidar->ClearDepth();
                                          //再计算新的深度图
                                          this->lidar->CalCoord(calibration_param.CameraExternal, calibration_param.cameraMatrix);
                                          //将计算得到的深度图下载到控制器
                                          this->depthMat = this->lidar->depthMat.clone();
                                      }
                                      else
                                      {
                                          depthFlag = false;
                                      }
                                  }
                                  else //如果用视频
                                  {
                                      //读取二进制深度图文件
                                      string depthmatsave_path = path_param.depthmatsave_path;
                                      if (depthmatsave_path[-1] != '/')
                                      {
                                          depthmatsave_path = depthmatsave_path + '/';
                                      }
                                      this->frameIndex++;
                                      string readName = depthmatsave_path + "Radar/" + to_string(this->frameIndex) + ".mb";
                                      this->depthMat = Utils::read(readName);
                                      //如果读取为空，则不使用激光雷达
                                      if (depthMat.empty())
                                      {
                                          depthFlag = false;
                                      }
                                  }
                              });
    // 線程等待
    readPoll.join();
    basePoll.join();
    dartPoll.join();
    depthPoll.join();

    //上面的深度图处理都已经获得了depthMat(但如果读取不到就不进行下面这一步，直接检测，并使用透视变换法)
    if (depthFlag)
    {
        //检测器的处理需要激光雷达的深度图，这里从控制器下载给检测器
        Mat depthMat = this->depthMat.clone();
        Mat depthMatDraw = this->depthMat.clone();
        this->locator->setDepthMat(depthMat);
        //再拷贝一份用于绘制框框
        this->locator->setDepthMatDraw(depthMatDraw);
    }

    // 检测器开始检测
    // 先將所有機器人的尋找狀態關閉
    for (int i = 0; i < robots.size(); i++)
    {
        robots[i]->setFindFlag(false);
    }
    this->mainDetector->Detect(this->frame, this->robots);
    if (baseCap != nullptr)
    {
        this->baseDetector->Detect(this->baseFrame, this->robots);
        this->baseDetector->Draw(this->robots);
    }
    //结果呈现绘制
    this->mainDetector->Draw(this->robots);
    if (this->path.empty())
    {
        // 定位器開始定位
        this->locator->LocateRobot(this->robots);
        this->locator->Draw(this->robots);
    }

    if (!depthMat.empty())
    {
        //获取绘制深度图（检测器到控制器）
        this->depthMatDraw = this->locator->getDepthMatDraw();
    }
    //裁判系统开了的话，把小地图信息和车间通信发送过去
    if (judgeFlag)
    {
        gameinfo.game_side = 1 - (int)judge->referee.robot_client_ID.robot_where;
        this->judge->sendMapData(this->locator->getMapPoint(), this->locator->getMapId());
        //this->judge->writeToReferee();
        //this->judge->testTransmit();
    }
    //行为树清空坐标点
    ClearLocation();
    //计算时间
    double te = (double)cv::getTickCount();
    double T = (te - ts) * 1000 / cv::getTickFrequency();
    //cout << "控制器运行一帧时间为：" << T << endl;
    return true;
}

/**
 * @brief 初始化相机
 * 
 */
void Radar::InitCamera()
{
    //开启相机，如果开启的不是相机，则开启视频
    if (this->path.empty())
    {
        // 主相机打开
        if (path_param.ismainmo)
        {
            this->mainCap = shared_ptr<cv::VideoCapture>(new VideoCapture(path_param.mainmono, CAP_V4L2));
        }
        else
        {
            this->mainCap = make_shared<RMVideoCapture>(path_param.mainno);
        }
        // 如果相机打开则设置，否则打开视频
        if (this->mainCap->isOpened())
        {
            if (path_param.ismainmo)
            {
                this->mainCap->set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G')); //设置为MJPG格式
                this->mainCap->set(CAP_PROP_FRAME_WIDTH, 1920);
                this->mainCap->set(CAP_PROP_FRAME_HEIGHT, 1080);
                // init - 曝光度
                this->mainCap->set(CAP_PROP_EXPOSURE, main_camera_param.cam_exposure);
            }
            else
            {
                // init - 曝光度
                this->mainCap->set(CAP_PROP_EXPOSURE, main_camera_param.cam_exposure);
                // init - 对比度
                this->mainCap->set(CAP_PROP_CONTRAST, main_camera_param.cam_contrast);
                // init - Gamma
                this->mainCap->set(CAP_PROP_GAMMA, main_camera_param.cam_gamma);
                // init - 设置颜色增益
                this->mainCap->set(CAP_PROP_XI_WB_KB, main_camera_param.cam_Bgain);
                this->mainCap->set(CAP_PROP_XI_WB_KG, main_camera_param.cam_Ggain);
                this->mainCap->set(CAP_PROP_XI_WB_KR, main_camera_param.cam_Rgain);
                this->mainCap->set(CAP_PROP_AUTO_WB, 0);
            }
        }
        else
        {
            this->mainCap->release();
            this->mainCap = shared_ptr<VideoCapture>(new VideoCapture(path));
        }

        // 基地监控相机打开
        if (path_param.isbasemo)
        {
            this->baseCap = shared_ptr<cv::VideoCapture>(new VideoCapture(path_param.othermono, CAP_V4L2));
            // 如果相机打开则设置，否则打开视频
            if (this->baseCap->isOpened())
            {
                if (path_param.isbasemo)
                {
                    this->baseCap->set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G')); //设置为MJPG格式
                    this->baseCap->set(CAP_PROP_FRAME_WIDTH, 1920);
                    this->baseCap->set(CAP_PROP_FRAME_HEIGHT, 1080);
                }
                else
                {
                    // // init - 曝光度
                    // this->baseCap->set(CAP_PROP_EXPOSURE, base_camera_param.cam_exposure);
                    // // init - 对比度
                    // this->baseCap->set(CAP_PROP_CONTRAST, base_camera_param.cam_contrast);
                    // // init - Gamma
                    // this->baseCap->set(CAP_PROP_GAMMA, base_camera_param.cam_gamma);
                    // // init - 设置颜色增益
                    // this->baseCap->set(CAP_PROP_XI_WB_KB, base_camera_param.cam_Bgain);
                    // this->baseCap->set(CAP_PROP_XI_WB_KG, base_camera_param.cam_Ggain);
                    // this->baseCap->set(CAP_PROP_XI_WB_KR, base_camera_param.cam_Rgain);
                    // this->baseCap->set(CAP_PROP_AUTO_WB, 0);
                }
            }
            else
            {
                this->baseCap->release();
                this->baseCap = shared_ptr<VideoCapture>(new VideoCapture(path));
            }
        }
        else
        {
            // this->baseCap = make_shared<RMVideoCapture>(path_param.otherno);
        }

        // 基地监控相机与飞镖相机
        if (!this->calibrationFlag)
        {

            // 飞镖相机
            this->dartCap = make_shared<RMVideoCapture>(path_param.dartno);
            if (this->dartCap->isOpened())
            {
                // init - 曝光度
                this->dartCap->set(CAP_PROP_EXPOSURE, dart_camera_param.cam_exposure);
                // init - 对比度
                this->dartCap->set(CAP_PROP_CONTRAST, dart_camera_param.cam_contrast);
                // init - Gamma
                this->dartCap->set(CAP_PROP_GAMMA, dart_camera_param.cam_gamma);
                // init - 设置颜色增益
                this->dartCap->set(CAP_PROP_XI_WB_KB, dart_camera_param.cam_Bgain);
                this->dartCap->set(CAP_PROP_XI_WB_KG, dart_camera_param.cam_Ggain);
                this->dartCap->set(CAP_PROP_XI_WB_KR, dart_camera_param.cam_Rgain);
                this->dartCap->set(CAP_PROP_AUTO_WB, 0);
            }
            else
            {
                this->dartCap->release();
                this->dartCap=nullptr;
                cout<<"can not load dart camera"<<endl;
                //this->dartCap = shared_ptr<VideoCapture>(new VideoCapture(path));
            }
        }
    }
    else
    {
        this->mainCap = shared_ptr<VideoCapture>(new VideoCapture());
        mainCap->open(path);
        if (mainCap->isOpened())
        {
            cout << "視頻成功打開" << endl;
        }
        else
        {
            cout << "視頻打開失敗" << endl;
        }
        // this->baseCap = shared_ptr<VideoCapture>(new VideoCapture(path));
        // this->dartCap = shared_ptr<VideoCapture>(new VideoCapture(path));
    }
}
/**
 * @brief 接收裁判系统数据线程函数
 * 
 * @param  threadFuture    future对象
 */
void Radar::ReceiveJudge(future<void> &threadFuture)
{
    while (threadFuture.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
        static int last_time;
        writeBlackBoard(this->judge);
        getStartTime = (double)cv::getTickCount();
        if (gameinfo.judge_time == last_time)
        {
            continue;
        }
        cout << "比赛剩下的时间(裁判系统):" << gameinfo.judge_time << endl;
        last_time = gameinfo.judge_time;
        gameinfo.remain_time = gameinfo.judge_time;
        //cout << "裁判系统修改了内置时钟:" << gameinfo.remain_time << endl;
        changeFlag = 1;
    }
}

/**
 * @brief 内部时钟线程函数
 * 
 * @param  threadFuture    future对象
 */
void Radar::InternalClock(future<void> &threadFuture)
{
    while (threadFuture.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
        double T = 0; //需要减去的时间
        if (getStartTime != -1 && gameinfo.remain_time == gameinfo.judge_time)
        {
            double te = (double)cv::getTickCount();
            T = (te - getStartTime) * 1000 / cv::getTickFrequency();
            //cout << "T:" << T << endl;
        }
        if (threadFuture.wait_for(std::chrono::microseconds((int)((1000 - T) * 1000))) == std::future_status::timeout)
        {
            if (changeFlag == 0)
            {
                //等待一秒比赛内置时间减去1；
                gameinfo.remain_time--;
            }
            changeFlag = 0;
            cout << "内置时钟的时间：" << gameinfo.remain_time << endl;
            //cout << "时间间隔为:" << (int)((1000 - T) * 1000) << endl;
        }
    }
}

/**
 * @brief 运行主要相机线程函数
 *
 * @param  threadFuture    future对象
 */
void Radar::RunMain(future<void> &threadFuture)
{
    while (threadFuture.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
        Mat picture;
        this->mainCap->read(picture);
        double ts = (double)cv::getTickCount();
        if (mainCapMutex.try_lock())
        {
            swap(picture, mainWait);
            mainCapMutex.unlock();
        }
        double te = (double)cv::getTickCount();
        double T = (te - ts) * 1000 / cv::getTickFrequency();
        // cout << "读取主相机一帧时间为：" << T << endl;
    }
}

/**
 * @brief 运行主要相机线程函数
 *
 * @param  threadFuture    future对象
 */
void Radar::RunBase(future<void> &threadFuture)
{
    while (threadFuture.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
        Mat picture;
        this->baseCap->read(picture);
        double ts = (double)cv::getTickCount();
        if (baseCapMutex.try_lock())
        {
            swap(picture, baseWait);
            baseCapMutex.unlock();
        }
        double te = (double)cv::getTickCount();
        double T = (te - ts) * 1000 / cv::getTickFrequency();
        // cout << "读取基地相机一帧时间为：" << T << endl;
    }
}

/**
 * @brief 读取主相机的函数
 *
 * @param  baseFrame       用于存储相片的Mat对象
 */
void Radar::ReadBase(Mat &baseFrame)
{
    baseCapMutex.lock();
    baseFrame = baseWait.clone();
    baseCapMutex.unlock();
}