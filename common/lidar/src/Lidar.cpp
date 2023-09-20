/**
 * @file Lidar.cpp
 * @author 梁俊玮 (1609205169@qq.com)
 * @brief 激光雷达库
 * @version 1.0
 * @date 2022-01-14
 * 
 * @copyright Copyright SCUT RobotLab(c) 2022
 * 
 */
#include "lidar_scan.h"
#include "Lidar.h"

/**
 * @brief 构造函数
 * 
 * @param  sensorHostname  传感器地址
 * @param  dataDestination 传感器要传输到的终点
 */
Lidar::Lidar(string sensorHostname, string dataDestination, cv::Size imgSize)
{
    this->imgSize = imgSize;
    if (sensorHostname != "")
    {
        //赋值
        this->sensorHostname = sensorHostname;
        this->dataDestination = dataDestination;
        //连接
        this->connect();
        //配置
        //this->configure();

        //转为XYZ点云
        //预计算表格，以便从范围有效计算点云
        this->lut = ouster::make_xyz_lut(this->info);
    }
    else
    {
        //解析元数据
        /*
        FileStorage fs("../Setting_file/PathParam.yml", FileStorage::READ);
        string metaPath;
        fs["meta_path"] >> metaPath;
        fs.release();
        */
        this->info = sensor::metadata_from_json(path_param.meta_path);
    }
    //构建初始深度图
    this->depthMat = cv::Mat(imgSize.width, imgSize.height, CV_64FC1, cv::Scalar::all(0));
    //解析数据
    this->prodLine = this->info.prod_line;         //获取线
    this->w = this->info.format.columns_per_frame; //每帧列数
    this->h = this->info.format.pixels_per_column; //每列像素数
    sensor::ColumnWindow columnWindow = info.format.column_window;
    this->columnWindowLength = (columnWindow.second - columnWindow.first + w) % w + 1;
    //输出相关信息
    cout << this->prodLine << "线" << endl;
    cout << "每帧列数：" << this->w << endl;
    cout << "每列像素数：" << this->h << endl;
    cout << "列窗口点一：" << columnWindow.first << endl;
    cout << "列窗口点二：" << columnWindow.second << endl;
    //构建扫描所需要用到的格式
    if (sensorHostname != "")
    {
        //准备扫描
        lsRead = shared_ptr<ouster::LidarScan>(
            new ouster::LidarScan{this->w, this->h, info.format.udp_profile_lidar});
        lsWrite = shared_ptr<ouster::LidarScan>(
            new ouster::LidarScan{this->w, this->h, info.format.udp_profile_lidar});

        auto packetFormat = sensor::get_format(this->info);
        this->batch = shared_ptr<ouster::ScanBatcher>(new ouster::ScanBatcher{this->w, packetFormat});

        //构建雷达数据包与imu数据包
        lidarBuf = std::shared_ptr<uint8_t[]>(
            new uint8_t[packetFormat.lidar_packet_size + 1]);
    }
}

/**
 * @brief 连接到激光雷达
 * 
 */
void Lidar::connect()
{
    //获得权柄
    this->handle = sensor::init_client(sensorHostname, dataDestination);
    if (!this->handle)
    {
        this->connectStatus = -1;
        cout << "连接失败" << endl;
        return;
    }
    else
    {
        this->connectStatus = 1;
        cout << "连接成功" << endl;
    }

    //获得元数据
    auto metadata = sensor::get_metadata(*(this->handle));
    //解析元数据
    this->info = sensor::parse_metadata(metadata);
}

/**
 * @brief 开启轮询扫描
 * 
 */
void Lidar::OpenScan(std::future<void> &threadFuture)
{
    //轮询
    //while (true)
    while (threadFuture.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
        // 轮询等待激光雷达数据
        sensor::client_state st = sensor::poll_client(*(this->handle));
        if (st & sensor::client_state::CLIENT_ERROR)
        {
            std::cerr << "客户端返回错误状态" << std::endl;
            std::exit(EXIT_FAILURE);
        }
        if (st & sensor::client_state::LIDAR_DATA)
        {
            double ts = (double)cv::getTickCount();
            if (sensor::read_lidar_packet(*(this->handle), lidarBuf.get(),
                                          get_format(this->info)))
            {
                if ((*batch)(lidarBuf.get(), *lsWrite))
                {
                    swap(lsRead, lsWrite); //将右边值交给左方
                }
            }
            double te = (double)cv::getTickCount();
            double T = (te - ts) * 1000 / cv::getTickFrequency();
            //cout << "扫描时间为：" << T << endl;
        }
    }
}

/**
 * @brief 计算深度图
 * 
 * @param  CEM              激光雷达到相机的外参矩阵
 * @param  CM               相机内参
 */
void Lidar::CalCoord(cv::Matx44d &CEM, cv::Matx33d &CM)
{
    double ts = (double)cv::getTickCount();
    Eigen::Ref<img_t<uint32_t>> range = (*this->lsRead).field(sensor::ChanField::RANGE);
    //计算三维坐标点
    for (int row = 0; row < range.rows(); row++)
    {
        for (int col = this->w / 3; col < this->w / 3 * 2; col++)
        {
            double nooffset_x = lut.direction(row * w + col, 0) * range(row, col);
            double nooffset_y = lut.direction(row * w + col, 1) * range(row, col);
            double nooffset_z = lut.direction(row * w + col, 2) * range(row, col);
            double x = nooffset_x == 0 ? 0 : nooffset_x + lut.offset(row * w + col, 0);
            double y = nooffset_y == 0 ? 0 : nooffset_y + lut.offset(row * w + col, 1);
            double z = nooffset_z == 0 ? 0 : nooffset_z + lut.offset(row * w + col, 2);
            cv::Matx41d lidar_point = {x, y, z, 1}; //4*1
            //激光雷达坐标系转为相机坐标系
            cv::Matx41d camera_point = CEM.inv() * lidar_point; //4*4 右乘 4*1
            //与像素坐标系建立关系
            cv::Matx31d correct_point = {camera_point(0, 0), camera_point(0, 1), camera_point(0, 2)};

            cv::Matx31d pixel = CM * correct_point;
            int u = (int)(pixel(0, 0) / pixel(0, 2));
            int v = (int)(pixel(0, 1) / pixel(0, 2));
            double cam_z = camera_point(0, 2);

            if (u <= 0 || v <= 0 || u >= this->imgSize.width || v >= this->imgSize.height)
            {
                continue;
            }
            if (cam_z < 0)
            {
                continue;
            }
            double original = *(depthMat.ptr<double>(v, u));
            if (original != 0)
            {
                *(depthMat.ptr<double>(v, u)) = min(original, cam_z);
            }
            else
            {
                *(depthMat.ptr<double>(v, u)) = cam_z;
            }
        }
    }
    //cv::imshow("img", depthMat); //显示深度图
    //cv::waitKey(1);

    //计算深度图时间
    double te = (double)cv::getTickCount();
    double T = (te - ts) * 1000 / cv::getTickFrequency();
    //cout << "深度图计算时间为：" << T << endl;
}