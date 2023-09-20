/**
 * @file Parameter.cpp
 * @author 梁俊玮 (1609205169@qq.com)
 * @brief 参数管理类
 * @version 1.0
 * @date 2021-11-27
 * 
 * @copyright Copyright SCUT RobotLab(c) 2022
 * 
 */
#include "Parameter.h"
#include <iostream>

CarParam::CarParam(const string &param)
{
    //读取文件
    FileStorage dlp(param, FileStorage::READ);

    //赋值
    dlp["DEVICE"] >> this->DEVICE;
    dlp["NMS_THRESH"] >> this->NMS_THRESH;
    dlp["BBOX_CONF_THRESH"] >> this->BBOX_CONF_THRESH;
    dlp["INPUT_W"] >> this->INPUT_W;
    dlp["INPUT_H"] >> this->INPUT_H;
    dlp["NUM_CLASSES"] >> this->NUM_CLASSES;
}
CarParam car_param("../Setting_file/CarParam.yml");

ArmorParam::ArmorParam(const string &param)
{
    //读取文件
    FileStorage dlp(param, FileStorage::READ);

    //赋值
    dlp["DEVICE"] >> this->DEVICE;
    dlp["NMS_THRESH"] >> this->NMS_THRESH;
    dlp["BBOX_CONF_THRESH"] >> this->BBOX_CONF_THRESH;
    dlp["INPUT_W"] >> this->INPUT_W;
    dlp["INPUT_H"] >> this->INPUT_H;
    dlp["NUM_CLASSES"] >> this->NUM_CLASSES;
}
ArmorParam armor_param("../Setting_file/ArmorParam.yml");

SortParam::SortParam(const string &param)
{
    //读取文件
    FileStorage sp(param, FileStorage::READ);

    //赋值
    sp["MAX_AGE"] >> this->MAX_AGE;
    sp["MIN_HITS"] >> this->MIN_HITS;
    sp["IOU_THRESHOLD"] >> this->IOU_THRESHOLD;
    sp["MIN_PROBABILITY"] >> this->MIN_PROBABILITY;
    sp["HIGH_PROBABILITY"] >> this->HIGH_PROBABILITY;
    sp["MIN_CLASSIFICATION"] >> this->MIN_CLASSIFICATION;
    sp["HIGH_RESET_TIME"] >> this->HIGH_RESET_TIME;
    sp["LOW_RESET_TIME"] >> this->LOW_RESET_TIME;
}
SortParam sort_param("../Setting_file/SortParam.yml");

CameraParam::CameraParam(const string &param)
{
    //读取文件
    FileStorage cam(param, FileStorage::READ);

    //赋值
    cam["camera_matrix"] >> this->cameraMatrix;
    cam["distortion_coefficients"] >> this->distCoeff;

    cam["cam_exposure"] >> this->cam_exposure;
    cam["cam_gamma"] >> this->cam_gamma;
    cam["cam_contrast"] >> this->cam_contrast;
    cam["cam_Bgain"] >> this->cam_Bgain;
    cam["cam_Ggain"] >> this->cam_Ggain;
    cam["cam_Rgain"] >> this->cam_Rgain;

    cam["image_width"] >> this->image_width;
    cam["image_height"] >> this->image_height;
}
CameraParam main_camera_param("../Setting_file/MainCameraParam.yml");
CameraParam base_camera_param("../Setting_file/BaseCameraParam.yml");
CameraParam dart_camera_param("../Setting_file/DartCameraParam.yml");

CalibrationParam::CalibrationParam(const string &param)
{
    //读取文件
    FileStorage cal(param, FileStorage::READ);

    //赋值
    cal["CameraExtrinsicMat"] >> this->CameraExternal;
    cal["CameraMat"] >> this->cameraMatrix;
    cal["pnp_scale_width"] >> this->pnp_scale_width;
    cal["pnp_scale_height"] >> this->pnp_scale_height;
    cal["new_range_thresh"] >> this->new_range_thresh;
    cal["back_range_thresh"] >> this->back_range_thresh;
    cal["cluster_thresh"] >> this->cluster_thresh;
    cal["cluster_times"] >> this->cluster_times;
}
CalibrationParam calibration_param("../Setting_file/CalibrationParam.yml");

PathParam::PathParam(const string &param)
{
    //读取文件
    FileStorage pap(param, FileStorage::READ);

    //赋值
    pap["mainno"] >> mainno;
    pap["otherno"] >> otherno;
    pap["mainmono"] >> mainmono;
    pap["othermono"] >> othermono;
    pap["dartno"] >> dartno;
    pap["islidar"] >> islidar;
    pap["isvideo"] >> isvideo;
    pap["ismainmo"] >> ismainmo;
    pap["isbasemo"] >> isbasemo;
    pap["lidarno"] >> lidarno;
    pap["video_path"] >> video_path;
    pap["calibrationmap_path"] >> calibrationmap_path;
    pap["redmap_path"] >> redmap_path;
    pap["bluemap_path"] >> bluemap_path;
    pap["carengine_path"] >> carengine_path;
    pap["armorengine_path"] >> armorengine_path;
    pap["meta_path"] >> meta_path;
    pap["area_path"] >> area_path;
    pap["btree_path"] >> btree_path;
    pap["videosave_path"] >> videosave_path;
    pap["depthmatsave_path"] >> depthmatsave_path;
}
PathParam path_param("../Setting_file/PathParam.yml");

BlackBoard gameinfo;
/**
 * @brief Get the Area object
 * 
 * @param  designArea      所有设计好的区域的轮廓信息
 */
void GetArea(vector<vector<Point>> &designArea)
{
    //读取区域图
    Mat map_area = imread(path_param.area_path);
    //可重复利用的轮廓容器
    vector<vector<Point>> contours;

    //公路区
    Mat Road;
    inRange(map_area, Scalar(0, 255, 255), Scalar(0, 255, 255), Road);
    findContours(Road, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    designArea.emplace_back(contours[0]);

    //读取飞坡数据
    Mat fly;
    inRange(map_area, Scalar(255, 255, 255), Scalar(255, 255, 255), fly);
    findContours(fly, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    designArea.emplace_back(contours[0]);

    //读取R2环形高地数据
    Mat R2;
    inRange(map_area, Scalar(0, 0, 255), Scalar(0, 0, 255), R2);
    findContours(R2, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    designArea.emplace_back(contours[0]);

    //中部环形高地
    Mat R2_middle;
    inRange(map_area, Scalar(0, 0, 250), Scalar(0, 0, 250), R2_middle);
    findContours(R2_middle, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    designArea.emplace_back(contours[0]);

    //读取R3
    Mat R3;
    inRange(map_area, Scalar(255, 0, 0), Scalar(255, 0, 0), R3);
    findContours(R3, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    designArea.emplace_back(contours[0]);

    //读取下方R4
    Mat R4;
    inRange(map_area, Scalar(0, 255, 0), Scalar(0, 255, 0), R4);
    findContours(R4, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    designArea.emplace_back(contours[0]);
}

/**
 * @brief 将区域规划写入黑板
 * 
 */
void WriteArea(void)
{
    vector<vector<Point>> designArea;
    GetArea(designArea);
    for (int i = 0; i < designArea.size(); i++)
    {
        gameinfo.enemy_density.area_list.emplace_back(designArea[i]);
    }
}

/**
 * @brief 清空位置信息
 * 
 */
void ClearLocation(void)
{
    for (int i = 0; i < 15; i++)
    {
        gameinfo.enemy_density.locations[i] = Point(-1, -1);
    }
}

/**
 * @brief 标志位复位
 * 
 */
void ResetFlags(void)
{
    gameinfo.timeflags.reset();
    gameinfo.timeflags = TimeFlags_ptr(new TimeFlags);
}