/**
 * @file SerialPort.cpp
 * @author 杨泽霖 (scut.bigeyoung@qq.com)
 * @brief Linux串口类
 * @version 2.0
 * @date 2018-12-08
 * 
 * @copyright Copyright South China Tiger(c) 2018
 * 
 */

#include "SerialPort.h"
#include <unistd.h>
#include <stdio.h>
#include <dirent.h>
#include <string.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <string>
#include "Logging.h"

using namespace std;

/**
 * @brief 构建一个新的串口实例
 * 将自动调用 open() 函数
 * @param baud_rate 波特率，以大写B开头
 * 拥有默认值 B115200
 */
SerialPort::SerialPort(int baud_rate)
{
    this->baud_rate = B115200;
    open();
}

/**
 * @brief 打开串口
 * 自动搜索所有可用设备，并尝试打开第一个
 */
void SerialPort::open()
{
    this->is_open = false;

    DIR *dir = nullptr;
    struct dirent *dire = nullptr;
    string file_name;
    const char dir_path[] = "/dev/";
    if ((dir = opendir(dir_path)) != nullptr)
    {
        while ((dire = readdir(dir)) != nullptr)
        {
            if (strstr(dire->d_name, "ttyUSB") != nullptr)
            {
                file_name = dire->d_name;
                break;
            }
        }
        closedir(dir);
    }

    if (file_name.empty())
    {
        ERROR_("找不到串口。");
        return;
    }
    else
        file_name = dir_path + file_name;

    INFO_("正在打开串口: " << file_name);
    fd = ::open(file_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); //非堵塞情况
    //fcntl(fd, F_SETFL,0);//恢复为阻塞状态

    if (fd == -1)
    {
        perror("\033[31m串口打开失败");
        printf("\033[0m");
        return;
    }
    tcgetattr(fd, &option);

    //修改所获得的参数
    option.c_iflag = 0;                 //原始输入模式
    option.c_oflag = 0;                 //原始输出模式
    option.c_lflag = 0;                 //关闭终端模式
    option.c_cflag |= (CLOCAL | CREAD); //设置控制模式状态，本地连接，接收使能
    option.c_cflag &= ~CSIZE;           //字符长度，设置数据位之前一定要屏掉这个位
    option.c_cflag &= ~CRTSCTS;         //无硬件流控
    option.c_cflag |= CS8;              //8位数据长度
    option.c_cflag &= ~CSTOPB;          //1位停止位
    option.c_cc[VTIME] = 0;
    option.c_cc[VMIN] = 0;//按裁判系统最小包长改的 72
    cfsetospeed(&option, baud_rate); //设置输入波特率
    cfsetispeed(&option, baud_rate); //设置输出波特率

    //设置新属性，TCSANOW：所有改变立即生效
    tcsetattr(fd, TCSANOW, &option);

    this->is_open = true;
}

SerialPort::~SerialPort()
{
    close();
}

void SerialPort::close()
{
    if (this->is_open)
        ::close(fd);
    this->is_open = false;
}

/**
 * @brief 写入数据
 * @param data 数据起始位置
 * @param len 想要写入的数据长度
 * @return bool 是否完整写入
 */
ssize_t SerialPort::write(void *data, size_t len)
{
    ssize_t len_result = -1;
    if (is_open)
    {
        tcflush(fd, TCOFLUSH); //清空，防止数据累积在缓存区
        len_result = ::write(fd, data, len);
    }

    if (len_result != static_cast<ssize_t>(len))
    {
        DEBUG_WARNING_("无法写入串口，重新打开中...");
        open();
    }
    else
    {
        DEBUG_INFO_("写入数据成功");
    }

    return len_result;
}

/**
 * @brief 读取数据
 * @param data 数据起始位置
 * @param len 想要读取的数据长度
 * @return ssize_t 读取到的长度
 */
ssize_t SerialPort::read(void *data, size_t len)
{
    ssize_t len_result = -1;
    if (is_open)
    {
        len_result = ::read(fd, data, len);
        tcflush(fd, TCIFLUSH);
    }

    if (len_result == -1)
    {
        DEBUG_WARNING_("无法读取串口，重新打开中...");
        open();
    }
    else if (len_result == 0)
    {
        //DEBUG_WARNING_("串口读取 空");
    }
    else
    {
        //DEBUG_INFO_("串口读取成功");
    }
    return len_result;
}

