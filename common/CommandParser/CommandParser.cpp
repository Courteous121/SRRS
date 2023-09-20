/**
 * @file CommandParser.cpp
 * @author 赵曦 (535394140@qq.com) (二次开发：梁俊玮 (1609205169@qq.com))
 * @brief 命令行参数
 * @version 1.0
 * @date 2022-02-01
 * 
 * @copyright Copyright SCUT RobotLab(c) 2022
 * 
 */

#include "CommandParser.h"

void recordPoint(int event, int x, int y, int flags, void *ustc)
{
    if (event == CV_EVENT_LBUTTONUP)
    {
        cout << "x:" << x << "y:" << y << endl;
    }
}

/**
 * @brief 构造函数
 * 
 * @param argc 
 * @param argv 
 */
CommandParser::CommandParser(int argc, char **argv)
{
    this->keys = "{ help      |       |打印帮助信息 }"
                 "{ input     |       |测试视频的路径 (不输入默认工业相机) }"
                 "{ output    |       |输出信息，包括: 协议数据、距离 }"
                 "{ c    |       |进行标定 }"
                 "{ g    |       |比賽，默認打開飛鏢相機、地圖、主相機 }"
                 "{ time      |       |计算运行时间 }"
                 "{ show      |       |显示测试图像、视频 }"
                 "{ map      |       |显示map }"
                 "{ depth      |       |显示depth }"
                 "{ base      |       |显示base }"
                 "{ src      |       |显示src }"
                 "{ dart      |       |显示dart }"
                 "{ record    |       |记录深度图 }"
                 "{ write     |       |保存视频至当前文件夹下 }";
    this->pParser = new CommandLineParser(argc, argv, keys);
    this->pWriter = nullptr;
    // 打印帮助信息
    if (this->pParser->has("help"))
    {
        this->pParser->printMessage();
        this->clear();
        exit(0);
    }
    // 显示测试图像、视频
    if (this->show())
    {
        namedWindow("Screen", 0);
        resizeWindow("Screen", Size(1920, 1080));
    }
    // 保存显示视频至当前文件夹下
    if (this->write() || this->game())
    {
        FileStorage fs("../Setting_file/PathParam.yml", FileStorage::READ);
        string videosave_path;
        fs["videosave_path"] >> videosave_path;
        if (videosave_path[-1] != '/')
        {
            videosave_path = videosave_path + '/';
        }
        rename((videosave_path + "Radar.old.old.old.avi").c_str(), (videosave_path + "Radar.old.old.old.old.avi").c_str());
        rename((videosave_path + "Radar.old.old.avi").c_str(), (videosave_path + "Radar.old.old.old.avi").c_str());
        rename((videosave_path + "Radar.old.avi").c_str(), (videosave_path + "Radar.old.old.avi").c_str());
        rename((videosave_path + "Radar.avi").c_str(), (videosave_path + "Radar.old.avi").c_str());
        // 這裏原有的編碼爲VideoWriter::fourcc('F', 'L', 'V', '1')，後綴爲avi
        this->pWriter = new VideoWriter(videosave_path + "Radar.avi",
                                        VideoWriter::fourcc('M', 'J', 'P', 'G'),
                                        45,
                                        Size(main_camera_param.image_width, main_camera_param.image_height),
                                        true);

        rename((videosave_path + "RadarDart.old.old.old.avi").c_str(), (videosave_path + "RadarDart.old.old.old.old.avi").c_str());
        rename((videosave_path + "RadarDart.old.old.avi").c_str(), (videosave_path + "RadarDart.old.old.old.avi").c_str());
        rename((videosave_path + "RadarDart.old.avi").c_str(), (videosave_path + "RadarDart.old.old.avi").c_str());
        rename((videosave_path + "RadarDart.avi").c_str(), (videosave_path + "RadarDart.old.avi").c_str());
        // 這裏原有的編碼爲VideoWriter::fourcc('F', 'L', 'V', '1')，後綴爲avi
        this->dWriter = new VideoWriter(videosave_path + "RadarDart.avi",
                                        VideoWriter::fourcc('M', 'J', 'P', 'G'),
                                        45,
                                        Size(dart_camera_param.image_width, dart_camera_param.image_height),
                                        true);
    }
}

/**
 * @brief Debug 模式
 * 
 * @param servo 视觉控制器
 * 
 * @return 是否结束循环
 */
void CommandParser::debug(Radar &servo)
{
    // parser_output
    this->parser_output(servo);
    // parser_show
    if (!this->parser_show(servo))
    {
        this->clear();
        exit(0);
    }
    // parser_write
    this->parser_write(servo);
    // parser_time;
    this->parser_time();
}

/**
 * @brief 析构函数
 */
CommandParser::~CommandParser()
{
    this->clear();
}

/**
 * @brief 清空
 */
void CommandParser::clear()
{
    // 清空命令行参数对象
    if (this->pParser != nullptr)
    {
        delete this->pParser;
        this->pParser = nullptr;
    }
    // 清空 VideoWriter 对象
    if (this->pParser != nullptr)
    {
        // 释放资源
        this->pWriter->release();
        this->dWriter->release();
        delete this->pWriter;
        delete this->dWriter;
        this->pWriter = nullptr;
        this->dWriter = nullptr;
    }
}

/**
 * @brief output
 * 
 * @param servo 视觉控制器
 * @param send_data 串口协议 -- 发送端
 */
void CommandParser::parser_output(Radar &servo)
{
    if (this->output()) // **输出信息**
    {
    }
}

/**
 * @brief time
 */
void CommandParser::parser_time()
{
    if (this->time())
    { // 准备数据
        static bool is_timing = false;
        static double start_time;
        static double end_time;
        static int num = 0;
        // 开始计时
        if (is_timing)
        {
            end_time = static_cast<double>(getTickCount());
            double time = (end_time - start_time) / getTickFrequency();
            num++;
            cout << "总运行时间为:   " << time << " s" << endl;
            cout << "平均运行时间为: " << time * 1000.f / num << " ms" << endl;
        }
        else
        {
            // 获取开始执行时间
            start_time = static_cast<double>(getTickCount());
            is_timing = true;
        }
    }
}

/**
 * @brief write
 * 
 * @param servo 视觉控制器
 */
void CommandParser::parser_write(Radar &servo)
{
    if (this->write() || this->game())
    {
        if (this->video_thread.joinable())
        {
            this->video_thread.detach();
        }
        this->video_thread = thread(&CommandParser::write_frame,
                                    this,
                                    std::ref<Mat>(servo.frame));
        if (this->dart_thread.joinable())
        {
            this->dart_thread.detach();
        }
        this->dart_thread = thread(&CommandParser::write_dart,
                                   this,
                                   std::ref<Mat>(servo.dartFrame));
    }

    if (this->record() || this->game())
    {
        if (this->depthmat_thread.joinable())
        {
            this->depthmat_thread.detach();
        }
        this->depthmat_thread = thread(&CommandParser::write_depthmat,
                                       this,
                                       std::ref<Mat>(servo.depthMat));
    }
}

/**
 * @brief 标定
 * 
 * @param  servo            视觉控制器
 */
void CommandParser::parser_calibration(Radar &servo)
{
    if (this->calibration())
    {
        //创建标定器
        Calibrator cbt;
        //开启标定
        cbt.MapCalibration("../Setting_file/MappingParam.yml", servo.getMainCap(), servo.getBaseCap());
    }
}

/**
 * @brief record each frame to VideoWriter
 * 
 * @param src image
 */
void CommandParser::write_frame(const Mat &src)
{
    // lock
    this->video_lock.lock();
    this->pWriter->write(src);
    // unlock
    this->video_lock.unlock();
}

/**
 * @brief   记录dart图
 * 
 * @param  dartFrame              dart图
 */
void CommandParser::write_dart(const Mat &dartFrame)
{
#ifdef DART_CORRECT
    if (game() && gameinfo.timeflags->dart_open_flag)
    {
        // lock
        if (!dartFrame.empty())
        {
            this->dart_lock.lock();
            this->dWriter->write(dartFrame);
            // unlock
            this->dart_lock.unlock();
        }
    }
#else {
    // lock
    if (!dartFrame.empty())
    {
        this->dart_lock.lock();
        this->dWriter->write(dartFrame);
        // unlock
        this->dart_lock.unlock();
    }
#endif
}

/**
 * @brief   记录深度图
 * 
 * @param  src              深度图
 */
void CommandParser::write_depthmat(const Mat &src)
{
    this->countFrame++;

    Utils::write(this->depthMatSavePath + "Radar/" + to_string(countFrame) + ".mb", src);
}

/**
 * @brief show
 * 
 * @param servo 视觉控制器
 * 
 * @return 是否结束循环
 */
bool CommandParser::parser_show(Radar &servo)
{
    // 显示测试图像、视频
    if (servo.getPath().empty())
    {
        if (servo.getBaseFlag() && (this->base() || this->show()))
        {
            imshow("Base", servo.getBaseDraw());
            resizeWindow("Base", servo.getBaseDraw().size() / 2);
        }
        if (servo.getDartFlag() && (this->dart() || this->show() || this->game()))
        {
#ifdef DART_CORRECT
            if (game() && !gameinfo.timeflags->dart_open_flag)
            {
                if (this->dartWindowFlag)
                {
                    destroyWindow("Dart");
                    this->dartWindowFlag = false;
                }
            }
            else
            {
                Mat img;
                resize(servo.getDart(), img, servo.getDart().size() / 2);
                imshow("Dart", img);
                this->dartWindowFlag = true;
            }
#else
            Mat img;
            resize(servo.getDart(), img, servo.getDart().size() / 2);
            imshow("Dart", img);
#endif
        }
        if (this->map() || this->show() || this->game() || true)
        {
            imshow("Map", servo.getMapDraw()); //地图
        }
        if (this->depth() || this->show())
        {
            // namedWindow("DepthMat", 0);

            imshow("DepthMat", servo.depthMatDraw); //深度图
            setMouseCallback("DepthMat", recordPoint, NULL);
        }
    }

    if (this->src() || this->show() || this->game())
    {
        Mat img;
        resize(servo.getSrcDraw(), img, servo.getSrcDraw().size() / 2);
        // namedWindow("Screen",0);
        
        imshow("Screen", img); //主屏幕
        setMouseCallback("Screen", recordPoint, NULL);
    }

    if (this->look())
    {
        // 延迟
        if (waitKey(1) == 27)
        {
            if (waitKey(0) == 27)
            {
                return false;
            }
        }
    }

    return true;
}
