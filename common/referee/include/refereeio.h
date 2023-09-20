#ifndef _REFEREE
#define _REFEREE

#include "referee.h"
#include "opencv2/opencv.hpp"
#include <vector>
#include "Parameter.h"
using namespace cv;

//裁判系统读取类
class refereeio
{
public:
    //裁判系统类
    referee_Classdef referee;
    //通信包定义
    PackToGroundRobotStratUnionDef _PackToSentry;
    PackToGroundRobotStratUnionDef _PackToEngineer;
    PackToGroundRobotStratUnionDef _PackToHero;
    PackToGroundRobotStratUnionDef _PackToInfantry;

    void writeToReferee(); //发送裁判系统数据
    bool readRefereeData() //读取裁判系统数据的IO口，读取之后直接读referee中对应成员值即可
    {
        const int LENGTH = 255;
        unsigned char read_buffer[LENGTH] = {0};
        ssize_t len_result = referee._SerialPort.read(read_buffer, LENGTH);
        referee.unPackDataFromRF(read_buffer, len_result);
        
        if (referee.GameState.game_progress != 15)
            return true;
        else
            return false;
    };
    void sendMapData(vector<Point> &point_mapb, vector<int> &id);
    void testTransmit();
};

//读取裁判系统数据并写入黑板
bool writeBlackBoard(shared_ptr<refereeio> judge);
#endif