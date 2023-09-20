/**
 * @file RMVideoCapture.h
 * @author 杨泽霖 (scut.bigeyoung@qq.com)
 * @brief 工业相机驱动头文件
 * @version 1.0
 * @date 2018-12-08
 * 
 * @copyright Copyright South China Tiger(c) 2018
 * 
 */

#pragma once

#include <opencv2/videoio.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/imgproc/types_c.h>
#include <memory>
#include <unistd.h>
#include "CameraApi.h"

class RMVideoCapture : public cv::VideoCapture
{
public:
    RMVideoCapture(const std::string &serial);
    RMVideoCapture(const std::string &serial, int decodeMethod, const std::vector<int> &decodeParam);

    virtual ~RMVideoCapture() override
    {
        release();
        if (g_pRgbBuffer)
            delete[] g_pRgbBuffer;
    };

    virtual bool set(int propId, double value) override;

    virtual double get(int propId) const override;

    bool open();

    virtual bool isOpened() const override { return iStatus == CAMERA_STATUS_SUCCESS; };

    virtual void release() override { CameraUnInit(hCamera); };

    virtual bool grab() override
    {
        return CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS;
    };

    //这个函数与OpenCV源码一致，实际上并没有重写。
    virtual bool read(cv::OutputArray image) override
    {
        if (grab())
        {
            retrieve(image, decodeMethodFlag);
        }
        else
        {
            reconnect();
            image.release();
        }
        return !image.empty();
    };

    virtual bool retrieve(cv::OutputArray image, int flag = 0) override;

    //这个函数与OpenCV源码一致，实际上并没有重写。
    virtual RMVideoCapture &operator>>(cv::Mat &image) override
    {
        read(image);
        return *this;
    };

    bool initLutTable(const std::vector<int> &lutTable);

    bool reconnect();

private:
    std::unique_ptr<std::string> serial_ptr; //当前正在使用相机的序列号
    CameraHandle hCamera;
    BYTE *g_pRgbBuffer = nullptr; //FIXME  名字有误 更改为***BgrBuff**
    BYTE *pbyBuffer = nullptr;
    IplImage *iplImage = nullptr;
    int iCameraCounts = 0;
    int cameraId;
    CameraSdkStatus iStatus;
    tSdkCameraDevInfo tCameraEnumList[3]; // 预留三个相机位
    tSdkCameraCapbility tCapability;      // 设备描述信息
    tSdkFrameHead sFrameInfo;             // 图像信息，包括了图像的大小
    int channel;

    int decodeMethodFlag = 0; //0-相机SDK读图 1-OpenCV读图 2,3......
    cv::Mat lookupTable1;     //单通道LUT表1

    /**曝光时间和白平衡的参数缓存*/
    double exposure_time; //曝光时间

    int gain = 1;    //全通道增益
    int Rgain = 100; //红色增益
    int Ggain = 100; //绿色增益
    int Bgain = 100; //蓝色增益

    /**图像处理特殊参数*/
    double gamma;            // Gamma
    double contrast;         // 对比度
    double saturation = 100; //饱和度
    double sharpness = 0;    //锐化

    using VideoCapture::open; //通过私有化明确告诉编译器不使用VideoCapture的open方法
};
