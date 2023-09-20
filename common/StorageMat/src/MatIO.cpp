#include "MatIO.h"
#include <fstream>
#include "tools.h"

using namespace std;
using namespace cv;

namespace Utils {
    /**
     * 保存mat头的关键信息
     */
    typedef struct {
        int rows;//行数
        int cols;//列数
        int type;//类型
    } MatHeader;
}

bool Utils::write(const std::string &fileName, const cv::Mat &src) {
    IF(!fileName.empty(), "文件名为空",false);
    IF(!src.empty(), "mat为空",false);
    IF(src.dims == 2,"不是二维Mat", false);

    Mat _src;
    // 保证mat是连续的
    if (!src.isContinuous()){
         _src = src.clone();
    } else{
        _src = src;
    }

    MatHeader matHeader{_src.rows, _src.cols, _src.type()};
//    printf("%d %d %d\n",matHeader.rows,matHeader.cols,matHeader.type);

    //打开文件
    ofstream out(fileName, ios::binary);
    IF(out.is_open(), "写入文件" + fileName + "失败",false);

    //写入文件类型，表示为这个类型的数据，长度为两字节
    char fileType[3] = "mb";
    out.write(fileType, 2*sizeof(char));

    //写入Mat头
    out.write((char *) &matHeader, sizeof(MatHeader));

    //写入数据
    //http://blog.csdn.net/dcrmg/article/details/52294259
    //https://www.cnblogs.com/wangguchangqing/p/4016179.html
    out.write((char *) _src.data, _src.rows * _src.step[0]);

    out.flush();
    out.close();

    return true;
}

cv::Mat Utils::read(const std::string &fileName) {
    //打开文件
    ifstream in(fileName, ios::binary);
    //IF(in.is_open(),"读取文件" + fileName + "失败",Mat());

    char fileType[3]={'\0','\0','\0'};//初始化一个默认值

    //读取前两字节
    in.read(fileType, 2*sizeof(char));

    //判断是否是"mb"这种文件
    //IF(strcmp(fileType, "mb") == 0,fileName + "非Mat类型的数据",Mat());

    //读取Mat头
    MatHeader matHeader{0, 0, 0};
    in.read((char *) &matHeader, sizeof(MatHeader));
//    printf("%d %d %d\n",matHeader.rows,matHeader.cols,matHeader.type);

    //初始化一个Mat
    Mat mat(matHeader.rows, matHeader.cols, matHeader.type);

    //写入数据
    in.read((char *) mat.data, mat.rows * mat.step[0]);

    in.close();
    return mat;
}
