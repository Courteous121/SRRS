/**
 * @file CommandParser.h
 * @author 赵曦 (535394140@qq.com) (二次开发：梁俊玮 (1609205169@qq.com))
 * @brief 命令行参数管理类头文件
 * @version 1.0
 * @date 2022-02-01
 * 
 * @copyright Copyright SCUT RobotLab(c) 2022
 * 
 */

#pragma once

#include "Radar.h"
#include "Calibrator.h"
#include "Parameter.h"
#include "MatIO.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>

class CommandParser
{
private:
    CommandLineParser *pParser;                             // 命令行参数对象
    bool dartWindowFlag = false;                            // 飛鏢窗口存在標誌位
    const char *keys;                                       // 关键字
    int countFrame = 0;                                     // 深度圖存儲計數器
    VideoWriter *pWriter;                                   // 录屏指针
    VideoWriter *dWriter;                                   // 录屏指针
    std::thread video_thread;                               // 录屏多线程对象
    std::thread dart_thread;                                // 录屏多线程对象
    std::thread depthmat_thread;                            // 录制深度图多线程对象
    std::mutex video_lock;                                  // 录屏互斥锁对象
    std::mutex dart_lock;                                   // 录屏互斥锁对象
    string depthMatSavePath = path_param.depthmatsave_path; // 深度图存储地址
public:
    CommandParser(int, char **);
    ~CommandParser();
    void debug(Radar &);
    inline string input() { return this->pParser->get<string>("input"); }
    inline bool calibration() { return this->pParser->has("c"); }
    void parser_calibration(Radar &);

private:
    void clear();
    void parser_output(Radar &);
    void parser_time();
    void parser_write(Radar &);
    bool parser_show(Radar &);
    void write_frame(const Mat &src);
    void write_dart(const Mat &dartFrame);
    void write_depthmat(const Mat &depthmat);
    inline bool output() { return this->pParser->has("output"); }
    inline bool time() { return this->pParser->has("time"); }
    inline bool write() { return this->pParser->has("write"); }
    inline bool record() { return this->pParser->has("record"); }
    inline bool show() { return this->pParser->has("show"); }
    inline bool base() { return this->pParser->has("base"); }
    inline bool dart() { return this->pParser->has("dart"); }
    inline bool src() { return this->pParser->has("src"); }
    inline bool map() { return this->pParser->has("map"); }
    inline bool depth() { return this->pParser->has("depth"); }
    inline bool game() { return this->pParser->has("g"); }
    inline bool look() { return (show() || base() || dart() || src() || map() || depth() || game()); }
};
