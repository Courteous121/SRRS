#ifndef __GETDATA_
#define __GETDATA_

#include "DataStruct.h"
// #include "ArmorDetector.h"

/**
 * @note 不同模式对应的帧头
 *       巡逻模式 -- 37
 *       追踪模式 -- 137
 *       射击模式 -- 141
 */

#define MergeCommand(__command_chassis, __command_holder, __command_modular) (__command_chassis*32+__command_holder*4+__command_modular)
#define GetCommandChassis(__command) (__command/32)
#define GetCommandHolder(__command)  ((__command%32)/4)
#define GetCommandModular(__command) (__command%4)

enum CommandChassis : uint8_t
{
    Patrol = 0x01,  //巡逻
    Left = 0x02,    //向左
    Right = 0x03,   //向右
    Randon = 0x04   //随机，配合云台的Track和Shoot
};

enum CommandHolder : uint8_t
{
    Spin = 0x01,            //自转
    Track = 0x02,           //追踪
    Single_Shoot = 0x03,    //单发模式
    More_Shoot = 0x04       //连发模式
};

enum CommandModular : uint8_t
{
    No_detected = 0x01, //桥头没有装甲板
    Detected = 0x02     //桥头有装甲板
};

//DataStruct getdata(short x, short y,uint8_t FPS);

#endif // !__GETDATA_