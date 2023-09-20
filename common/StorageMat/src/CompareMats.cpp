
#include "CompareMats.h"
#include "tools.h"
#include <fstream>

using namespace cv;
using namespace std;
using namespace Utils;

CompareMats::CompareMats(const cv::Mat &mat0, const cv::Mat &mat1):
        _sameCount(0), _differentCount(0), _same(true) {
    ASSERT(!mat0.empty(), "mat0指定的图片为空");
    ASSERT(!mat1.empty(), "mat1指定的图片为空");
    ASSERT(mat0.size() == mat1.size(), "mat0和mat1大小不一致");
    ASSERT(mat0.type() == mat1.type(), "mat0和mat1的类型不一致");
    switch (mat0.type()) {
        case CV_8UC1: {
            vector<Mat> mats;
            for (int i = 0; i < 3; i++) {
                mats.push_back(mat0);
            }
            merge(mats, _mask);
            break;
        }
        case CV_8UC3:
            _mask = mat0.clone();
            break;
        case CV_8UC4:
            cvtColor(mat0, _mask, COLOR_BGR2BGR555);
            break;
        default:
            _mask = Mat(mat0.size(), CV_8UC3);
            break;
    }
    compare(mat0, mat1);
}

std::string CompareMats::report() {
    if (_same)
        return "是否相同:是, 相同点数量:" + to_string(_sameCount) + ", 不同点数量:" + to_string(_differentCount);
    else
        return "是否相同:否, 相同点数量:" + to_string(_sameCount) + ", 不同点数量:" + to_string(_differentCount);

}

/**
 * 比较Mat是否相同;
 * @param mat1 mat1
 * @param mat2 mat2
 */
void CompareMats::compare(const Mat &mat1, const Mat &mat2) {
    /**
     * 这个部分利用了非常巧妙的方式来比较不同类型的mat;不管mat里面存储的是单通道
     * 还是多通道，通道分量的类型uchar、int、float还是double，mat中的每个元素所
     * 占内存总是uchar(8位)的整数倍，因此，可以把该元素看成一个uchar数组。箬要比
     * 较两个mat中对应位置的元素是否相等，可以先求得各自指向该元素的uchar数组的首
     * 地址，然后用一个for循环来比较这两个数组，若完全相同，则这两个mat中对应位置
     * 的元素是相等的，否则，不相等。
     */

    //参考链接：http://blog.csdn.net/dcrmg/article/details/52294259
    size_t elemSize = mat1.elemSize();//mat(i,j)，也就是一个元素，所占的字节数
    uchar *ptrCols1 = nullptr;//mat1行首指针
    uchar *ptrCols2 = nullptr;//mat2行首指针
    uchar *array1 = nullptr;//mat1元数数组首指针
    uchar *array2 = nullptr;//mat1元数数组首指针

    for (int i = 0; i < mat1.rows; i++) {
        //获取每行行首的指针
        ptrCols1 = const_cast<uchar *>(mat1.ptr<uchar>(i));
        ptrCols2 = const_cast<uchar *>(mat2.ptr<uchar>(i));
        for (int j = 0; j < mat1.cols; j++) {
            //获取(i,j)元素对应数组的指针
            array1 = ptrCols1 + j * elemSize;
            array2 = ptrCols2 + j * elemSize;
            //判断是否相等
            bool tag = true;
            for (int ii = 0; ii < elemSize; ii++) {
                if (array1[ii] != array2[ii]) {
                    tag = false;
                    break;
                }
            }
            //
            if (tag) {//相等
                _sameCount++;
            } else {//不等
                _differentCount++;
                _differentPoints.emplace_back(Point(i, j));

                _mask.at<Vec3b>(i, j) = Vec3b(0, 255, 255);
            }
        }
    }
    ptrCols1 = nullptr;
    ptrCols2 = nullptr;
    array1 = nullptr;
    array2 = nullptr;

    if (_differentCount > 0) {
        _same = false;
    }
}

void CompareMats::saveReport(std::string fileName) {
    ofstream o(fileName + ".txt");

    ASSERT(o.is_open(), "新建文件失败");

    o << report() << endl;

    if (_differentPoints.empty()) {
        o.close();
        return;
    }

    o << "不同点坐标:" << endl;

    for (int i = 0; i < _differentPoints.size(); i++) {
        o << _differentPoints[i] << ",\t";
        if ((i + 1) % 10 == 0)//存10个点就换行
            o << endl;
    }
    o.close();

    imwrite(fileName + ".bmp", _mask);

    cout << "报告已保存到" + fileName + ".txt和" + fileName + ".bmp" << endl;
}

