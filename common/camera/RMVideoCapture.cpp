/**
 * @file RMVideoCapture.cpp
 * @author 杨泽霖 (scut.bigeyoung@qq.com)
 * @brief 工业相机驱动
 * @version 1.0
 * @date 2018-12-08
 * 
 * @copyright Copyright SCUT RobotLab(c) 2021
 * 
 */

#include <opencv2/imgproc.hpp>
#include "RMVideoCapture.h"
#include "Logging.h"

using namespace std;
using namespace cv;

RMVideoCapture::RMVideoCapture(const std::string &serial)
{
    this->decodeMethodFlag = 0; //旧的相机调用函数默认就是SDK读图 标志位0
    this->serial_ptr = std::make_unique<std::string>(serial);
    for (int i = 0; i < 3; ++i)
    {
        this->tCameraEnumList[i] = {0};
    }
    open();
}

RMVideoCapture::RMVideoCapture(const string &serial, int decodeMethod, const std::vector<int> &decodeParam)
{
    this->decodeMethodFlag = decodeMethod;
    if (decodeMethodFlag == 1)
    {
        initLutTable(decodeParam);
    } //opencv读图--LUT查表
    if (decodeMethodFlag == 2)
    {
        ;
    } //opencv读图--无LUT查表

    // if( decodeMethodFlag==3 ) {init*****(decodeParam);}
    // if( decodeMethodFlag==4 ) {init*****(decodeParam);}
    // PS->其他读取解码方式 如上所述
    this->serial_ptr = std::make_unique<std::string>(serial);
    for (int i = 0; i < 3; ++i)
    {
        this->tCameraEnumList[i] = {0};
    }
    open();
}

bool RMVideoCapture::set(int propId, double value)
{
    switch (propId)
    {
    case CAP_PROP_EXPOSURE:
        exposure_time = value;
        CameraSetAeState(hCamera, false);              //关闭自动曝光
        CameraSetExposureTime(hCamera, exposure_time); //曝光时间设置
        return true;
    case CAP_PROP_GAIN:
        gain = value;
        CameraSetAnalogGain(hCamera, gain);
    case CAP_PROP_AUTO_WB:
        CameraSetGain(hCamera, Rgain, Ggain, Bgain);
        return true;
    case CAP_PROP_XI_WB_KB:
        Bgain = value;
        return true;
    case CAP_PROP_XI_WB_KG:
        Ggain = value;
        return true;
    case CAP_PROP_XI_WB_KR:
        Rgain = value;
        return true;
    case CAP_PROP_GAMMA:
        gamma = value;
        CameraSetGamma(hCamera, gamma);
        return true;
    case CAP_PROP_CONTRAST:
        contrast = value;
        CameraSetContrast(hCamera, contrast);
        return true;
    case CAP_PROP_SATURATION:
        saturation = value;
        CameraSetSaturation(hCamera, saturation);
        return true;
    case CAP_PROP_SHARPNESS:
        sharpness = value;
        CameraSetSharpness(hCamera, sharpness);
        return true;
    default:
        DEBUG_ERROR_(__FILE__ << ", line" << __LINE__ << ": 尝试设置未定义变量。");
        return false;
    }
}

double RMVideoCapture::get(int propId) const
{
    switch (propId)
    {
    case CAP_PROP_EXPOSURE:
        return exposure_time;
    case CAP_PROP_GAIN:
        return gain;
    default:
        DEBUG_ERROR_(__FILE__ << ", line" << __LINE__ << ": 尝试获取未定义变量。");
        return 0;
    }
}

bool RMVideoCapture::open()
{
    iCameraCounts = 3;
    // 第一次置零相机状态用于判断是否检测到相机
    iStatus = -1;
    iplImage = nullptr;
    CameraSdkInit(1);

    // 枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(tCameraEnumList, &iCameraCounts);
    DEBUG_INFO_("相机枚举状态：" << CameraGetErrorString(iStatus));
    DEBUG_INFO_("相机数量：" << iCameraCounts);
    // 没有连接设备
    if (iCameraCounts == 0)
    {
        ERROR_("找不到相机。");
        return false;
    }
    // 第二次置零相机状态用于判断相机是否匹配到序列号
    iStatus = -1;
    // 根据序列号进行比对
    for (size_t index = 0; index < 3; ++index)
    { // 倘若第二位为初始值0，直接退出循环(tCameraEnumList内的相机信息是依次填充的)
        if (tCameraEnumList[index].acSn[1] == 0)
            break;
        if (tCameraEnumList[index].acSn == *serial_ptr)
        {
            // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
            iStatus = CameraInit(&tCameraEnumList[index], -1, -1, &hCamera);
            DEBUG_INFO_("成功打开相机：" << tCameraEnumList[index].acSn);
            break;
        }
    }
    // 倘若未匹配到序列号则默认打开第一个相机
    if (iStatus == CAMERA_STATUS_FAILED)
    {
        iStatus = CameraInit(&tCameraEnumList[0], -1, -1, &hCamera);
        DEBUG_INFO_("成功打开相机：" << tCameraEnumList[0].acSn);
    }

    //初始化失败
    DEBUG_INFO_("相机初始化状态：" << CameraGetErrorString(iStatus));
    if (iStatus != CAMERA_STATUS_SUCCESS)
    {
        ERROR_("相机初始化失败。");
        return false;
    }

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera, &tCapability);
    if (!g_pRgbBuffer)
        g_pRgbBuffer = new BYTE[tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3];
    CameraPlay(hCamera);

    if (tCapability.sIspCapacity.bMonoSensor)
    {
        channel = 1;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
    }
    else
    {
        channel = 3;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
    }

    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */

    return true;
}

bool RMVideoCapture::retrieve(OutputArray image, int flag)
{
    int64 timeStamp = getTickCount(); //TODO 获取物理世界时间戳

    if (channel != 1 && channel != 3)
    {
        ERROR_("相机通道数错误：" << channel << "可能是相机没有成功初始化");
        image.assign(Mat());
        return false;
    }
    //SDK参数处理
    if (flag == 0)
    {
        CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);
        if (g_pRgbBuffer)
        {
            image.assign(Mat(
                cvSize(sFrameInfo.iWidth, sFrameInfo.iHeight),
                sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                g_pRgbBuffer));
            CameraReleaseImageBuffer(hCamera, pbyBuffer);
            *reinterpret_cast<int64 *>(image.getMat().data) =
                timeStamp; // 时间戳写在图像中前3个像素中
            return true;
        }
        else
        {
            image.assign(Mat());
            ERROR_("SDK解码有误：" << decodeMethodFlag << "可能是相机设置问题");
            return false;
        }
    }
    //OpenCV-LUT查表
    if (flag == 1)
    {
        Mat bayerImg{Size(sFrameInfo.iWidth, sFrameInfo.iHeight), CV_8U, pbyBuffer};
        if (channel == 1)
        {
            image.assign(bayerImg);
        }
        if (channel == 3)
        {
            Mat bgrImg{Size(sFrameInfo.iWidth, sFrameInfo.iHeight), CV_8U}; // 此处拷贝开销无需优化->需保证视频流的正常->优化拷贝将造成画面黑影 ; 此坑别踩cvtColor
            LUT(bayerImg, lookupTable1, bgrImg);                            //针对bayerImg查表再转化成三通道图 运算量小但效果相同
            cvtColor(bgrImg, image, COLOR_BayerRG2BGR);                     //cvtColor多线程等效果很好 适合单核薄弱机器
            // TODO 寻找更快的cvtcolor运算 读图的主要开销 或者多线程读图 或者parralfor执行LUT查表？尚未测试
        }
        CameraReleaseImageBuffer(hCamera, pbyBuffer);
        *reinterpret_cast<int64 *>(image.getMat().data) = timeStamp; //时间戳写在图像中前3个像素中
        return true;
    }

    //OpenCV读图--无LUT查表
    if (flag == 2)
    {
        Mat bayerImg{Size(sFrameInfo.iWidth, sFrameInfo.iHeight), CV_8U, pbyBuffer};
        if (channel == 1)
        {
            image.assign(bayerImg);
        }
        if (channel == 3)
        {
            Mat bgrImg;                                    // 此处拷贝开销无需优化->需保证视频流的正常->优化拷贝将造成画面黑影 ; 此坑别踩cvtColor
            cvtColor(bayerImg, bgrImg, COLOR_BayerRG2BGR); //cvtColor多线程等效果很好 适合单核薄弱机器
            image.assign(bgrImg);
        }
        CameraReleaseImageBuffer(hCamera, pbyBuffer);
        *reinterpret_cast<int64 *>(image.getMat().data) = timeStamp; //时间戳写在图像中前3个像素中
        return true;
    }
    //iplImage
    if (flag == 3)
    {
        if (iplImage)
        {
            cvReleaseImageHeader(&iplImage);
        }
        iplImage = cvCreateImageHeader(cvSize(sFrameInfo.iWidth, sFrameInfo.iHeight), IPL_DEPTH_8U, channel);
        cvSetData(iplImage, g_pRgbBuffer, sFrameInfo.iWidth * channel); //此处只是设置指针，无图像块数据拷贝，不需担心转换效率
        image.assign(cvarrToMat(iplImage));                             //这里只是进行指针转换，将IplImage转换成Mat类型
        CameraReleaseImageBuffer(hCamera, pbyBuffer);
        int64 timeStamp = getTickCount();
        return true;
    }

    //OpenCV读图--无LUT查表
    if (flag == 4)
    {
        //内存拷贝 执行了三次
        Mat bayerImg{Size(sFrameInfo.iWidth, sFrameInfo.iHeight), CV_8U, pbyBuffer};

        uchar bgrImgArray[sFrameInfo.iWidth * sFrameInfo.iHeight];

        if (channel == 3)
        {
            Mat bgrImg;                                    // 此处拷贝开销无需优化->需保证视频流的正常->优化拷贝将造成画面黑影 ; 此坑别踩cvtColor
            cvtColor(bayerImg, bgrImg, COLOR_BayerRG2BGR); //cvtColor多线程等效果很好 适合单核薄弱机器
            image.assign(bgrImg);
        }

        CameraReleaseImageBuffer(hCamera, pbyBuffer);
        *reinterpret_cast<int64 *>(image.getMat().data) = timeStamp; //时间戳写在图像中前3个像素中
        return true;
    }

    ERROR_("解码方式标志位错误：" << decodeMethodFlag << "可能是标志位设置错误");
    image.assign(Mat());
    return false;
}

bool RMVideoCapture::initLutTable(const vector<int> &lutTable)
{

    //TODO 没有必要专门设置一个initLutTable的方案
    if (lutTable.size() != 256)
    {
        ERROR_("LUT查表参数数量有误");
        return false;
    }
    uchar table1[256];
    for (int i = 0; i < 256; i++)
    {
        table1[i] = uchar(lutTable[i]);
    }
    lookupTable1 = Mat(1, 256, CV_8U, table1).clone();
    return true;
}

bool RMVideoCapture::reconnect()
{
    ERROR_("相机重连");
    release();
    sleep(1);
    open();
    set(CAP_PROP_EXPOSURE, exposure_time);
    set(CAP_PROP_AUTO_WB, 0);
    return true;
}
