/**
 * @file Parameter.h
 * @author 梁俊玮 (1609205169@qq.com)
 * @brief 参数管理类
 * @version 1.0
 * @date 2021-11-27
 * 
 * @copyright Copyright SCUT RobotLab(c) 2022
 * 
 */
#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// 车辆检测学习参数
struct CarParam
{
    int DEVICE;             // 使用什么 设备（gpu）
    float NMS_THRESH;       // nms阈值
    float BBOX_CONF_THRESH; // bbox置信度阈值
    int INPUT_W;            // 输入图片宽度
    int INPUT_H;            // 输入图片高度
    int NUM_CLASSES;        // 分类数

    CarParam(const string &param);
};

// 装甲板检测学习参数
struct ArmorParam
{
    int DEVICE;             // 使用什么 设备（gpu）
    float NMS_THRESH;       // nms阈值
    float BBOX_CONF_THRESH; // bbox置信度阈值
    int INPUT_W;            // 输入图片宽度
    int INPUT_H;            // 输入图片高度
    int NUM_CLASSES;        // 分类数

    ArmorParam(const string &param);
};

// 目标跟踪算法参数
struct SortParam
{
    int MAX_AGE;             // 最大帧数没有匹配
    int MIN_HITS;            // 最小击中次数
    double IOU_THRESHOLD;    // iou阈值
    double MIN_PROBABILITY;  // 最小冻结概率
    double HIGH_PROBABILITY; // 最大冻结概率
    int MIN_CLASSIFICATION;  // 最小连续相同分类
    int HIGH_RESET_TIME;     // 高概率冻结重置时间
    int LOW_RESET_TIME;      // 低概率冻结重置时间

    SortParam(const string &param);
};

// 相机参数
struct CameraParam
{
    Mat cameraMatrix; // 相机内参
    Mat distCoeff;    // 畸变参数
    int image_width;  // 相机宽度和高度
    int image_height; // 相机宽度和高度
    int cam_exposure; // 相机曝光
    int cam_gamma;    // 相机 Gamma 值
    int cam_contrast; // 相机对比度
    int cam_Bgain;    // 相机蓝色增益
    int cam_Ggain;    // 相机绿色增益
    int cam_Rgain;    // 相机红色增益

    CameraParam(const string &param);
};

// 标定参数
struct CalibrationParam
{
    Matx44d CameraExternal;   // 相机到激光雷达外参
    Matx33d cameraMatrix;     // 相机内参
    double pnp_scale_width;   // pnp标定的宽度缩放
    double pnp_scale_height;  // pnp标定的高度缩放
    double back_range_thresh; // 背景深度阈值
    double new_range_thresh;  // 最后的前景深度阈值
    double cluster_thresh;    // 聚类阈值
    int cluster_times;        // 聚类的次数

    CalibrationParam(const string &param);
};

// 地址相关参数
struct PathParam
{
    int islidar;                // 是否使用激光雷达
    int isvideo;                // 是否使用测试视频
    int ismainmo;               // 是否使用主相机模组
    int isbasemo;               // 是否使用副相机模组
    int mainmono;               // 主相机模组序列号
    int othermono;              // 副相机模组序列号
    string mainno;              // 主相机序列号
    string otherno;             // 副相机序列号
    string dartno;              // 飞镖相机序列号
    string lidarno;             // 激光雷达序列号
    string video_path;          // 测试视频地址
    string calibrationmap_path; // 标定地图地址
    string redmap_path;         // 红方地图原图地址
    string bluemap_path;        // 蓝方地图原图地址
    string carengine_path;      // 车辆推理引擎
    string armorengine_path;    // 装甲板推理引擎
    string meta_path;           // 激光雷达元数据存储地址
    string area_path;           // 规划区域图
    string btree_path;          // 行为树地址
    string videosave_path;      // 录制视频保存地址
    string depthmatsave_path;   // 深度信息保存地址

    PathParam(const string &param);
};

// 外部引用
extern CarParam car_param;
extern ArmorParam armor_param;
extern SortParam sort_param;
extern CameraParam main_camera_param;
extern CameraParam base_camera_param;
extern CameraParam dart_camera_param;
extern CalibrationParam calibration_param;
extern PathParam path_param;

typedef struct
{
    //红方血量
    int r_outpost_HP = 1500; //前哨站血量
    int r_sentry_HP = 600;   //哨兵血量
    int r_base_HP = 5000;    //基地血量
    int R1 = 150;
    int R2 = 500;
    int R3 = 100;
    int R4 = 100;
    int R5 = 100;
    //蓝方血量
    int b_outpost_HP = 1500; //前哨站血量
    int b_sentry_HP = 600;   //哨兵血量
    int b_base_HP = 5000;    //基地血量
    int B1 = 150;
    int B2 = 500;
    int B3 = 100;
    int B4 = 100;
    int B5 = 100;
} HPList;

typedef struct
{
    //位置信息，来自映射（现在还没写，假装它有）
    Point locations[15] = {
        Point(-1, -1),
        Point(-1, -1),
        Point(-1, -1),
        Point(-1, -1),
        Point(-1, -1),
        Point(-1, -1),
        Point(-1, -1),
        Point(-1, -1),
        Point(-1, -1),
        Point(-1, -1),
        Point(-1, -1),
        Point(-1, -1),
        Point(-1, -1),
        Point(-1, -1),
        Point(-1, -1)};
    //区域位置(暂定6个，后面可以改)
    vector<vector<Point>> area_list;
    //公路区敌人密度，代号0
    //对方飞坡起始区密度，代号1
    //R2环形高地敌人密度，代号2
    //R2环形高地楼梯敌人密度，代号2
    //R3区域敌人密度，代号4
    //R4区域敌人密度，代号5
    int all_density[5];
    int fly_time;
} EnemyDensity;

typedef struct
{
    bool front_mines_flag = false;         // 前面矿石开始掉落标志位
    bool back_mines_flag = false;          // 后面矿石开始掉落标志位
    bool front_minee_flag = false;         // 前面矿石结束掉落标志位
    bool back_minee_flag = false;          // 后面矿石结束掉落标志位
    bool small_buff_can_flag = false;      // 小符可打标志位
    bool big_buff_can_flag = false;        // 大符可打标志位
    bool mine_drop_flag = false;           // 矿石准备掉落标志位
    bool watcher_defense_buff_flag = true; // 哨兵防御增益标志位
    bool watcher_die_flag = false;         // 哨兵阵亡标志位
    bool outpost_invincible_flag = true;   // 前哨站无敌状态标志位
    bool outpost_die_flag = false;         // 前哨站阵亡标志位
    bool base_invincible_flag = true;      // 基地无敌状态标志位
    bool virtual_shield_flag = false;      // 虚拟护盾标志位
    bool buff_flag = false;                // 能量机关标志位
    bool danger_flag = false;              // 危险标志位
    bool fly_flag = false;                 // 飞坡标志位
    bool in_buff_flag = false;             // 处于buff状态
    bool openitself_flag = false;          // 矿灯自动打开
    bool launch_flag = false;              // 发射标志位
    bool dart_open_flag = false;           // 飞镖舱门打开标志位
} TimeFlags;
using TimeFlags_ptr = shared_ptr<TimeFlags>;

typedef struct
{
    int game_status = 0;                                    // 比赛状态，1为比赛，0为调试
    int remain_time = 420;                                  // 比赛剩余时间(内置时钟)
    int dart_open_time = 0;                                 // 飞镖仓门开启时间
    int judge_time = 420;                                   // 裁判系统时钟
    int buff_count = 0;                                     // 能量机关计时器
    int buff_start_time = 0;                                // buff开始时间
    int game_side;                                          // 属于哪一边,蓝0红1
    int game_stage = 1;                                     // 游戏阶段
    int weak_enemy = -1;                                    // 残血敌人id
    int weak_teammate = -1;                                 // 残血队友id
    int mine_time = 0;                                      // 矿石掉落提醒
    int mine_count = 20;                                    // 矿石掉落计时器
    int buff_open_count = 20;                               // buff开启倒计时
    vector<int> weak_list;                                  // 虚弱列表
    TimeFlags_ptr timeflags = TimeFlags_ptr(new TimeFlags); // 各种时间标志位
    HPList HP_list;                                         // 血量记录表
    EnemyDensity enemy_density;                             // 敌人密度表
} BlackBoard;
extern BlackBoard gameinfo;

//获得区域的轮廓信息
void GetArea(vector<vector<Point>> &designArea);
//将区域规划写入黑板
void WriteArea(void);
//清空位置信息
void ClearLocation(void);
//标志位复位
void ResetFlags(void);
