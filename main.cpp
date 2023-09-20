/**
 * @file main.cpp
 * @author 梁俊玮 (1609205169@qq.com)
 * @brief 主函数
 * @version 1.0
 * @date 2022-01-12
 * 
 * @copyright Copyright SCUT RobotLab(c) 2022
 * 
 */

#include "Parameter.h"
#include "Radar.h"
#include "CommandParser.h"

// 主函数
int main(int argc, char *argv[])
{
    // --------------------【数据准备】--------------------
    // 命令行参数管理对象
    CommandParser parser(argc, argv);
    // 创建视觉控制器
    // Radar servo("/home/duke/Radar/Loadings/test1.mp4", "",parser.calibration());
    Radar servo("", "os-122139001023.local",parser.calibration());
    // Radar servo("", "",parser.calibration());
    parser.parser_calibration(servo);
    // --------------------【循环处理】--------------------
    while (1)
    {
        if (!servo.Run())
            continue;
        // Debug 模式
        parser.debug(servo);
    }

    return 0;
}
