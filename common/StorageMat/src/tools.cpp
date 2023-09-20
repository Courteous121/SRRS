#include <iostream>
#include <string>
#include "tools.h"

using namespace std;

void error(const string &filePath, const string &function, int line, const string &info) {
    //获取文件名
    unsigned long pos = filePath.find_last_of('/');
    string filename(filePath.substr(pos + 1));
    string err = "文件:" + filename + ",函数:" + function + ",行:" + to_string(line) + ">>  " + info;
    //抛出错误
    throw runtime_error(err);
}

void warning(const string &filePath, const string &function, int line, const string &info) {
    //获取文件名
    unsigned long pos = filePath.find_last_of('/');
    string filename(filePath.substr(pos + 1));
    string err = "文件:" + filename + ",函数:" + function + ",行:" + to_string(line) + ">>  " + info;
    //输出警告
    cerr << err << endl;
}
