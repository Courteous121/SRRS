#include "Locator.h"

Locator::Locator()
{
#ifdef LIDAR_DAMAGE
    this->damageFlag = true;
#endif
    //读取地图照片
    if (gameinfo.game_side)
    {
        this->map = imread(path_param.redmap_path);
    }
    else
    {
        this->map = imread(path_param.bluemap_path);
    }
    this->calibrationMap = imread(path_param.calibrationmap_path);
    //读取变换矩阵
    //读取文件
    FileStorage fs("../Setting_file/MappingParam.yml", FileStorage::READ);
    //赋值
    fs["WARPMATRIX"] >> this->warpMatrix;
    fs["WARPMATRIXS"] >> this->warpMatrixs;
    fs["PNPMATRIXS"] >> this->pnpMatrixs;
    //释放文件
    fs.release();
}

void Locator::LocateRobot(vector<Robot_ptr> &robots)
{
    this->mapPoint.clear();
    this->mapId.clear();
    for (auto robot : robots)
    {
        if (!robot->getFindFlag())
        {
            continue;
        }
        //获取机器人的图像坐标
        Rect imgLocation = (*robot).getImgLoaction();
        Point resPoint;
        if (this->damageFlag)
        {
            Point dectPoint = Point((int)(imgLocation.x + imgLocation.width / 2), (int)(imgLocation.y + imgLocation.height / 2));
            DoMap(warpMatrixs, dectPoint, resPoint);
        }
        else if (robot->getBaseFlag())
        {
            Point dectPoint = Point((int)(imgLocation.x + imgLocation.width / 2), (int)(imgLocation.y + imgLocation.height / 2));
            double xmap, ymap;
            PeTrans(warpMatrix, xmap, ymap, dectPoint);
            resPoint = Point(ymap, map.size().width - xmap);
        }
        else
        {
            //深度代表值
            double zRepresent;
            //使用k聚类
            zRepresent = KRepresent(imgLocation);
            //计算目标坐标
            Matx31d pixels = {(double)(imgLocation.x + imgLocation.width / 2) * zRepresent, (double)(imgLocation.y + imgLocation.height / 2) * zRepresent, zRepresent};
            Matx31d camPoints = calibration_param.cameraMatrix.inv() * pixels;
            Matx41d cam4Points = {camPoints(0, 0), camPoints(1, 0), camPoints(2, 0), 1};
            Matx41d worldPoints = pnpMatrixs.inv() * cam4Points;

            resPoint = Point((int)(worldPoints(1, 0) * calibration_param.pnp_scale_height), map.size().height - (int)(worldPoints(0, 0) * calibration_param.pnp_scale_width));
        }

        // 將結果保存起來
        this->mapPoint.emplace_back(resPoint);
        this->mapId.emplace_back(robot->getID());
        //完善机器人的图片坐标信息（这里涉及一个进制转换的问题,scale是地图像素/真实长度(毫米为单位)）
        (*robot).setMapLocation(resPoint);
        //存储行为树位置信息
        gameinfo.enemy_density.locations[(*robot).getID()] = resPoint;
    }
}

double Locator::KRepresent(Rect &imgLocation)
{
    /******************先定三个初始点（计算平均值，以平均值前后0.5m和平均值为三个点）******************/
    //先计算所有深度的平均值
    //所有深度的总和
    double all_range = 0;
    //点云个数
    double range_num = 0;
    for (int row = imgLocation.y + (int)(imgLocation.height / 2); row < imgLocation.y + imgLocation.height; row++)
    {
        for (int col = imgLocation.x; col < imgLocation.x + imgLocation.width; col++)
        {
            //获取深度值
            double nowRange = *(depthMat.ptr<double>(row, col));
            if (nowRange == 0)
            {
                continue;
            }
            all_range += nowRange;
            range_num++;
        }
    }
    //所有深度的平均值
    double range_mean = all_range / range_num;
    //cout << "range_mean:" << range_mean << endl;

    //定义三个聚类中心
    double clusters[3] = {range_mean - 1.5, range_mean, range_mean + 1.5};

    /**********************************进入循环（超越次数和符合最低标准则退出）*************************/
    int times = 0;
    bool continueFlag = true;
    while (times <= calibration_param.cluster_times && continueFlag)
    {
        /****************************************分配三个线程任务*******************************************/
        clusterAll[0] = 0;
        clusterAll[1] = 0;
        clusterAll[2] = 0;

        clusterNum[0] = 0;
        clusterNum[1] = 0;
        clusterNum[2] = 0;

        //不同线程的距离
        int threadColDistance = (int)(imgLocation.width / 3);
        //线程1
        thread thread1 = thread([&]
                                {
                                    int leftEdge[4] = {0};
                                    leftEdge[0] = imgLocation.x;
                                    leftEdge[1] = imgLocation.x + threadColDistance;
                                    leftEdge[2] = imgLocation.y + (int)(imgLocation.height / 2);
                                    leftEdge[3] = imgLocation.y + imgLocation.height;
                                    this->KMeans(leftEdge, clusters);
                                });
        //线程2
        thread thread2 = thread([&]
                                {
                                    int middleEdge[4] = {0};
                                    middleEdge[0] = imgLocation.x + threadColDistance;
                                    middleEdge[1] = imgLocation.x + threadColDistance * 2;
                                    middleEdge[2] = imgLocation.y;
                                    middleEdge[3] = imgLocation.y + imgLocation.height;
                                    this->KMeans(middleEdge, clusters);
                                });
        //线程2
        thread thread3 = thread([&]
                                {
                                    int rightEdge[4] = {0};
                                    rightEdge[0] = imgLocation.x + threadColDistance * 2;
                                    rightEdge[1] = imgLocation.x + imgLocation.width;
                                    rightEdge[2] = imgLocation.y + (int)(imgLocation.height / 2);
                                    rightEdge[3] = imgLocation.y + imgLocation.height;
                                    this->KMeans(rightEdge, clusters);
                                });
        thread1.join();
        thread2.join();
        thread3.join();
        /****************************************计算新的聚类中心*******************************************/
        double nowClusterFront = 0;
        double nowClusterMiddle = 0;
        double nowClusterBack = 0;
        if (clusterNum[0] != 0)
        {
            nowClusterFront = clusterAll[0] / clusterNum[0];
        }
        if (clusterNum[1] != 0)
        {
            nowClusterMiddle = clusterAll[1] / clusterNum[1];
        }
        if (clusterNum[2] != 0)
        {
            nowClusterBack = clusterAll[2] / clusterNum[2];
        }
        if (fabs(nowClusterFront - clusters[0]) < calibration_param.cluster_thresh &&
            fabs(nowClusterMiddle - clusters[1]) < calibration_param.cluster_thresh &&
            fabs(nowClusterBack - clusters[2]) < calibration_param.cluster_thresh)
        {
            continueFlag = false;
            //cout << "因为时间符合条件退出" << endl;
        }
        clusters[0] = nowClusterFront;
        clusters[1] = nowClusterMiddle;
        clusters[2] = nowClusterBack;
        times++;
    }
    // cout << "cluster_front:" << cluster_front << endl;
    // cout << "cluster_middle:" << cluster_middle << endl;
    // cout << "cluster_back:" << cluster_back << endl;
    // cout << "cluster_front_num:" << cluster_front_num << endl;
    // cout << "cluster_middle_num:" << cluster_middle_num << endl;
    // cout << "cluster_back_num:" << cluster_back_num << endl;
    //以中间的聚类中心作为zRepresent

    int allNum = 0;
    double zRepresent = 0;
    for (int i = 0; i < 3; i++)
    {
        if (clusterNum[i] == 0)
        {
            continue;
        }
        allNum += clusterNum[i];
        zRepresent += (clusterNum[i] * clusters[i]);
    }
    zRepresent = zRepresent / allNum * 1000;
    // cout << "zRepresent:" << zRepresent << endl;

    return zRepresent;
}

void Locator::KMeans(int edge[4], double clusters[3])
{
    // 設定邊界
    int leftEdge = edge[0];
    int rightEdge = edge[1];
    int topEdge = edge[2];
    int bottomEdge = edge[3];
    // 設定簇中心
    for (int row = topEdge; row < bottomEdge; row++)
    {
        for (int col = leftEdge; col < rightEdge; col++)
        {
            //获取深度值
            double nowRange = *(depthMat.ptr<double>(row, col));
            if (nowRange == 0)
            {
                continue;
            }
            if (fabs(clusters[0] - nowRange) < fabs(clusters[1] - nowRange))
            {
                if (fabs(clusters[0] - nowRange) < fabs(clusters[2] - nowRange))
                {
                    //前面最小
                    clusterAll[0] += nowRange;
                    clusterNum[0]++;
                }
                else
                {
                    //后面最小
                    clusterAll[2] += nowRange;
                    clusterNum[2]++;
                }
            }
            else
            {
                if (fabs(clusters[1] - nowRange) < fabs(clusters[2] - nowRange))
                {
                    //中间最小
                    clusterAll[1] += nowRange;
                    clusterNum[1]++;
                }
                else
                {
                    //后面最小
                    clusterAll[2] += nowRange;
                    clusterNum[2]++;
                }
            }
        }
    }
}

/**
 * @brief 透视变换
 * 
 * @param  warpMatrix       变换矩阵
 * @param  xmap             结果x
 * @param  ymap             结果y
 * @param  dectPoint       需要变换的检测点
 */
void Locator::PeTrans(Mat &warpMatrix, double &xmap, double &ymap, Point &dectPoint)
{
    Mat org = (Mat_<double>(3, 1) << dectPoint.x, dectPoint.y, 1);
    Mat res = warpMatrix * org;
    double *x = res.ptr<double>(0, 0);
    double *y = res.ptr<double>(0, 1);
    double *z = res.ptr<double>(0, 2);
    xmap = *x / *z;
    ymap = *y / *z;
    //cout << xmap << ',' << ymap << endl;
}

/**
 * @brief 透视变换
 * 
 * @param  mapImg          输入地图图片
 * @param  warpMatrixs     变换矩阵列列表
 * @param  pointDect       需要检测的点集
 * @param  result           结果存储容器
 */
void Locator::DoMap(vector<Mat> &warpMatrixs, Point &pointDect, Point &result)
{
    //低透视变换
    Mat lowWarpMatrix = warpMatrixs[0];
    //中透视变换
    Mat middleWarpMatrix = warpMatrixs[1];
    //高透视变换
    Mat highWarpMatrix = warpMatrixs[2];

    //结果存储
    //vector<Point> result;
    //vector<Point> correct_result;
    //vector<Point> correct_two_result;

    //区域信息获取
    vector<vector<Point>> designArea;
    GetArea(designArea);
    //做透视变换的
    double xmap;
    double ymap;
    //低中高透视变换
    PeTrans(highWarpMatrix, xmap, ymap, pointDect);
    if (pointPolygonTest(designArea[2], Point2f(xmap, ymap), false) == 1)
    {
        result = Point(ymap, map.size().width - xmap);
    }
    else if (pointPolygonTest(designArea[4], Point2f(xmap, ymap), false) == 1)
    {
        result = Point(ymap, map.size().width - xmap);
    }

    PeTrans(middleWarpMatrix, xmap, ymap, pointDect);
    if (pointPolygonTest(designArea[3], Point2f(xmap, ymap), false) == 1)
    {
        result = Point(ymap, map.size().width - xmap);
    }
    else if (pointPolygonTest(designArea[5], Point2f(xmap, ymap), false) == 1)
    {
        result = Point(ymap, map.size().width - xmap);
    }
    else if (pointPolygonTest(designArea[0], Point2f(xmap, ymap), false) == 1)
    {
        result = Point(ymap, map.size().width - xmap);
    }
    else if (pointPolygonTest(designArea[1], Point2f(xmap, ymap), false) == 1)
    {
        result = Point(ymap, map.size().width - xmap);
    }

    PeTrans(lowWarpMatrix, xmap, ymap, pointDect);
    result = Point(ymap, map.size().width - xmap);
}

// 繪製函數
void Locator::Draw(vector<Robot_ptr> &robots)
{
    this->depthMatDraw = this->depthMat.clone();
    this->mapDraw = this->map.clone();
    for (auto robot : robots)
    {
        if (robot->getMapLocation().x == 0 && robot->getMapLocation().y == 0)
        {
            continue;
        }
        if (!robot->getFindFlag())
        {
            continue;
        }
        rectangle(this->depthMatDraw, robot->getImgLoaction(), Scalar(0, 0, 255), 10);

        char text[256];
        //这里为小地图图标绘画
        if ((*robot).getID() >= 0 && (*robot).getID() < 5) //对蓝方的处理
        {
            sprintf(text, "%s ", class_names[(*robot).getID()]);
            if ((*robot).getID() == 0) //对蓝方英雄的处理
            {
                cv::putText(this->mapDraw, text, cv::Point((*robot).getMapLocation().x, (*robot).getMapLocation().y + 1),
                            cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 165, 0), 3);
                circle(this->mapDraw, (*robot).getMapLocation(), 5, Scalar(255, 165, 0), 7);
            }
            else
            {
                cv::putText(this->mapDraw, text, cv::Point((*robot).getMapLocation().x, (*robot).getMapLocation().y + 1),
                            cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 2);
            }
            circle(this->mapDraw, (*robot).getMapLocation(), 5, Scalar(255, 0, 0), 5);
        }
        else if ((*robot).getID() >= 5 && (*robot).getID() < 10) //对红方的处理
        {
            sprintf(text, "%s ", class_names[(*robot).getID()]);
            if ((*robot).getID() == 5) //对红方英雄的处理
            {
                cv::putText(this->mapDraw, text, cv::Point((*robot).getMapLocation().x, (*robot).getMapLocation().y + 1),
                            cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 165, 255), 3);
                circle(this->mapDraw, (*robot).getMapLocation(), 5, Scalar(0, 165, 255), 7);
            }
            else
            {
                cv::putText(this->mapDraw, text, cv::Point((*robot).getMapLocation().x, (*robot).getMapLocation().y + 1),
                            cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
            }
            circle(this->mapDraw, (*robot).getMapLocation(), 5, Scalar(0, 0, 255), 5);
        }
        else if ((*robot).getID() >= 10 && (*robot).getID() < 15) //对灰色的处理
        {
            sprintf(text, "%s ", class_names[(*robot).getID()]);
            cv::putText(this->mapDraw, text, cv::Point((*robot).getMapLocation().x, (*robot).getMapLocation().y + 1),
                        cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(96, 96, 96), 3);
            circle(this->mapDraw, (*robot).getMapLocation(), 5, Scalar(96, 96, 96), 5);
        }
        else //对未进行分类的处理
        {
            circle(this->mapDraw, (*robot).getMapLocation(), 5, Scalar(0, 114, 189), 5);
        }
    }
}