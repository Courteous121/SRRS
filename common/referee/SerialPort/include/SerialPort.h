#ifndef __SERIALPORT_
#define __SERIALPORT_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>   //文件控制定义
#include <termios.h> //终端控制定义
#include <memory>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <vector>

#define S_TIMEOUT 1

using namespace std;

class SerialPort
{
public:
  SerialPort(int baud_rate = B115200);

  ~SerialPort();

  void open();  // 打开串口
  void close(); // 关闭串口
  bool isOpen() { return is_open; };

  // TODO 使用头尾校验无法实现透明传输
  // 需要改进串口协议
  // template <typename T>
  //迁移到Referee/refereeio.h
  ssize_t read(void *data, size_t len);
  template <typename T>
  bool writeStruct(T data_struct)
  {
    ssize_t len_result = this->write(&data_struct, sizeof(data_struct));
    return (sizeof(data_struct) == len_result);
  }

private:
  int fd;
  struct termios option;
  bool is_open = false;
  int baud_rate;
  ssize_t write(void *data, size_t len);
};

using serial_ptr = shared_ptr<SerialPort>;

#endif // !__SERIALPORT_
