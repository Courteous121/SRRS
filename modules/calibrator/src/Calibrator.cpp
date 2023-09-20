/**
 * @file Calibration.cpp
 * @author 梁俊玮 (1609205169@qq.com)
 * @brief 标定小工具
 * @version 1.0
 * @date 2022-01-12
 * 
 * @copyright Copyright SCUT RobotLab(c) 2022
 * 
 */
#include "Calibrator.h"

// 图像点集回调函数
void RecordImg(int event, int x, int y, int flags, void *ustc)
{
    static int num = 0;
    int pointClassNum = 2;
#ifdef LIDAR_DAMAGE
    pointClassNum = 3;
#endif
    if (event == CV_EVENT_RBUTTONUP)
    {
        if (allImagePoint.size() == 0)
        {
            return;
        }
        else
        {
            cout << "消除了一个" << endl;
            allImagePoint.pop_back();
            num--;
        }
    }
    if (allImagePoint.size() == pointClassNum * 4)
    {
        cout << "已标记完成" << endl;
        return;
    }
    if (event == CV_EVENT_LBUTTONUP)
    {
        cout << x << "," << y << endl;
        allImagePoint.emplace_back(Point2f(x, y));
        num++;
    }
    if (num == 4)
    {
        cout << "已标记完一组" << endl;
        num = 0;
    }
}

// 物体点集回调函数
void RecordObj(int event, int x, int y, int flags, void *ustc)
{
    static int num = 0;
    int pointClassNum = 2;
#ifdef LIDAR_DAMAGE
    pointClassNum = 3;
#endif
    if (event == CV_EVENT_RBUTTONUP)
    {
        if (allObjectPoint.size() == 0)
        {
            return;
        }
        else
        {
            cout << "消除了一个" << endl;
            allObjectPoint.pop_back();
            num--;
        }
    }
    if (allObjectPoint.size() == pointClassNum * 4)
    {
        cout << "已标记完成" << endl;
        return;
    }
    if (event == CV_EVENT_LBUTTONUP)
    {
        cout << x << "," << y << endl;
        allObjectPoint.emplace_back(Point2f(x, y));
        num++;
    }
    if (num == 4)
    {
        cout << "已标记完一组" << endl;
        num = 0;
    }
}

/**
 * @brief 构造函数
 * 
 */
Calibrator::Calibrator()
{
    // 設定組數,有激光雷達的情況下應該爲2，沒有激光雷達的情況下爲3
    this->pointClassNum = 2;
#ifdef LIDAR_DAMAGE
    this->pointClassNum = 3;
#endif
    // 读取标定用的地图
    this->map = imread(path_param.calibrationmap_path);
}

/**
 * @brief 析构函数
 * 
 */
Calibrator::~Calibrator()
{
}

/**
 * @brief PNP解算，用于激光雷达
 * 
 */
void Calibrator::PnpMap(vector<Point2f> &imagePoint, vector<Point2f> &objectPoint, Matx44d &pnpMatrixs)
{
    vector<Point3f> objectPoints;
    for (int i = 0; i < objectPoint.size(); i++)
    {
        objectPoints.emplace_back(Point3f(objectPoint[i].x / calibration_param.pnp_scale_width, objectPoint[i].y / calibration_param.pnp_scale_height, 0));
    }

    //定义旋转向量与平移向量
    Mat rVec;
    Mat tVec;
    //pnp解算
    solvePnP(objectPoints, imagePoint, main_camera_param.cameraMatrix, main_camera_param.distCoeff, rVec, tVec, false, SOLVEPNP_P3P);

    Mat rotM = Mat::zeros(3, 3, CV_64FC1);
    Rodrigues(rVec, rotM); //罗德里格斯变换

    pnpMatrixs(3, 3) = 1;
    for (int i = 0; i < 3; i++)
    {
        // 赋值最后一行
        pnpMatrixs(3, i) = 0;
        // 赋值旋转矩阵
        for (int j = 0; j < 3; j++)
        {
            pnpMatrixs(i, j) = *(rotM.ptr<double>(i, j));
        }
        // 赋值平移向量
        pnpMatrixs(i, 3) = *(tVec.ptr<double>(i, 0));
    }
}

/**
 * @brief 映射准备
 * 
 * @param  imagePoint      相片点集
 * @param  objectPoint     物体点集
 * @param  warpMatrixs     变换矩阵列表
 */
void Calibrator::MapPrepare(Mat &img, vector<vector<Point2f>> &imagePoint, vector<vector<Point2f>> &objectPoint, vector<Mat> &warpMatrixs)
{
    //低中高的图像点
    vector<Point2f> lowImagePoints = imagePoint[0];
    vector<Point2f> middleImagePoints = imagePoint[1];
    vector<Point2f> highImagePoints = imagePoint[2];
    //低中高的物体点
    vector<Point2f> lowObjectPoints = objectPoint[0];
    vector<Point2f> middleObjectPoints = objectPoint[1];
    vector<Point2f> highObjectPoints = objectPoint[2];
    //低透视变换
    Mat lowWarpMatrix = getPerspectiveTransform(lowImagePoints, lowObjectPoints);
    //中透视变换
    Mat middleWarpMatrix = getPerspectiveTransform(middleImagePoints, middleObjectPoints);
    //高透视变换
    Mat highWarpMatrix = getPerspectiveTransform(highImagePoints, highObjectPoints);

    //变换矩阵存储
    warpMatrixs.emplace_back(lowWarpMatrix);
    warpMatrixs.emplace_back(middleWarpMatrix);
    warpMatrixs.emplace_back(highWarpMatrix);

    /*
    //需要检测的点
    vector<Point> pointDect;
    pointDect.emplace_back(Point(417, 345));
    pointDect.emplace_back(Point(346, 324));
    pointDect.emplace_back(Point(182, 333));
    pointDect.emplace_back(Point(1054, 363));
    pointDect.emplace_back(Point(862, 317));
    */
}

/**
 * @brief 映射标定
 * 
 * @param  path             保存路径
 * @param  cap              相机
 * @param  map              地图
 * @return true    标定成功
 * @return false    标定失败
 */
bool Calibrator::MapCalibration(const string &path, shared_ptr<VideoCapture> mainCap, shared_ptr<VideoCapture> baseCap)
{
    // 定义帧和窗口
    cout << "开始标定" << endl;
    namedWindow("picture", 0);

    // ------------------------------看這裏---------------------------------------
    // 小地圖預先標註點，注意先後順序
    allObjectPoint.emplace_back(Point(161, 186));
    allObjectPoint.emplace_back(Point(267, 60));
    allObjectPoint.emplace_back(Point(288, 287));
    allObjectPoint.emplace_back(Point(399, 86));
    // ------------------------------看這裏---------------------------------------

    // 進行點組的分離
    vector<Point2f> tempImagePoint;
    //读取图片进行标定
    while (mainCap != nullptr && mainCap->read(this->src))
    {
        //可视化标记点
        this->ShowProcess(allImagePoint, allObjectPoint);
        //展示图像
        resizeWindow("picture", src.size() / 2);
        imshow("picture", this->drawSrc);
        imshow("map", this->drawMap);
        //设置回调函数
        setMouseCallback("picture", RecordImg, NULL);
        setMouseCallback("map", RecordObj, NULL);

        if (waitKey(1) == 27)
        {
            if (waitKey(0) == 27)
            {
                destroyWindow("picture");
                for (auto imagePoint : allImagePoint)
                {
                    tempImagePoint.emplace_back(imagePoint);
                }
                allImagePoint.clear();
                break;
            }
        }
    }

    //读取图片进行标定
    while (baseCap != nullptr && baseCap->read(this->src))
    {
        //可视化标记点
        this->ShowProcess(allImagePoint, allObjectPoint);
        //展示图像
        resizeWindow("picture", src.size() / 2);
        imshow("picture", this->drawSrc);
        imshow("map", this->drawMap);
        //设置回调函数
        setMouseCallback("picture", RecordImg, NULL);
        setMouseCallback("map", RecordObj, NULL);

        if (waitKey(1) == 27)
        {
            if (waitKey(0) == 27)
            {
                destroyWindow("picture");
                for (auto imagePoint : allImagePoint)
                {
                    tempImagePoint.emplace_back(imagePoint);
                }
                allImagePoint.clear();
                break;
            }
        }
    }
    destroyAllWindows();

    //判断是否标记成功
    if (tempImagePoint.size() != pointClassNum * 4 || allObjectPoint.size() != pointClassNum * 4)
    {
        if (tempImagePoint.size() != 4 || allObjectPoint.size() != 4)
        {
            return false;
        }
    }
    this->MapCalibrationCore(path, tempImagePoint, allObjectPoint);
    return true;
}

/**
 * @brief 映射标定核心
 * 
 * @param  path             保存路径
 */
void Calibrator::MapCalibrationCore(const string &path, vector<Point2f> &allImagePoint, vector<Point2f> &allObjectPoint)
{
    // 先定好三個變量
    vector<Mat> warpMatrixs;
    Mat warpMatrix;
    Matx44d pnpMatrixs;
    // 讀取歷史數據來初始化
    FileStorage bfs(path, FileStorage::READ);
    bfs["WARPMATRIXS"] >> warpMatrixs;
    bfs["WARPMATRIX"] >> warpMatrix;
    bfs["PNPMATRIXS"] >> pnpMatrixs;
    bfs.release();

    //图像点集
    vector<vector<Point2f>> imagePoint;
    //物体点集
    vector<vector<Point2f>> objectPoint;
    vector<Point2f> temp;
    //拆成2组
    for (int i = 0; i < allImagePoint.size(); i++)
    {
        temp.emplace_back(allImagePoint[i]);
        if (i % 4 == 3)
        {
            imagePoint.emplace_back(temp);
            temp.clear();
        }
    }
    temp.clear();
    for (int i = 0; i < allObjectPoint.size(); i++)
    {
        temp.emplace_back(allObjectPoint[i]);
        if (i % 4 == 3)
        {
            objectPoint.emplace_back(temp);
            temp.clear();
        }
    }

    // 只有主相機時的處理
    if (imagePoint.size() == 1 && objectPoint.size() == 1)
    {
        pointClassNum = 1;
        cout << "只有主相機的標定情況！" << endl;
    }
    //透视变换矩阵获取
    if (allObjectPoint.size() == pointClassNum * 4 && allImagePoint.size() == pointClassNum * 4)
    {
#ifdef LIDAR_DAMAGE
        MapPrepare(map, imagePoint, objectPoint, warpMatrixs);
        cout << "映射變換標定成功(損壞版本)" << endl;
#endif
        if (pointClassNum * 4 == 8)
        {
            warpMatrix = getPerspectiveTransform(imagePoint[1], objectPoint[1]);
            cout << "映射變換標定成功" << endl;
        }
    }

    //激光雷达变换矩阵
    //外参矩阵
    PnpMap(imagePoint[0], objectPoint[0], pnpMatrixs);
    cout << "PNP標定成功" << endl;
    //存储
    //开启文件
    FileStorage fs(path, FileStorage::WRITE);
    //存储变换矩阵
    fs << "WARPMATRIXS" << warpMatrixs;
    fs << "WARPMATRIX" << warpMatrix;
    fs << "PNPMATRIXS" << pnpMatrixs;
    //文件关闭
    fs.release();
    cout << "標定數據存儲完畢！" << endl;
}

/**
 * @brief 可视化标记点
 * 
 * @param  allImagePoint  全部图片点
 * @param  allObjectPoint 全部地图点
 */
void Calibrator::ShowProcess(vector<Point2f> &allImagePoint, vector<Point2f> &allObjectPoint)
{
    //深拷贝图片
    this->drawSrc = this->src.clone();
    this->drawMap = this->map.clone();

    //画src点
    for (int i = 0; i < allImagePoint.size(); i++)
    {
        circle(this->drawSrc, allImagePoint[i], 10, Scalar(255 * (i > 3 ? 0 : 1), 255 * (i > 7 ? 0 : 1), 255 * (i > 11 ? 0 : 1)), 2);
        circle(this->drawSrc, allImagePoint[i], 8, Scalar(0, 0, 0), -1);
    }

    //画map点
    for (int i = 0; i < allObjectPoint.size(); i++)
    {
        circle(this->drawMap, allObjectPoint[i], 7, Scalar(255 * (i > 3 ? 0 : 1), 255 * (i > 7 ? 0 : 1), 255 * (i > 11 ? 0 : 1)), 2);
        circle(this->drawMap, allObjectPoint[i], 5, Scalar(0, 0, 0), -1);
    }
}
