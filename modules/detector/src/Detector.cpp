/**
 * @file Detector.cpp
 * @author 梁俊玮 (1609205169@qq.com)
 * @brief 机器人检测
 * @version 1.0
 * @date 2022-01-12
 * 
 * @copyright Copyright SCUT RobotLab(c) 2022
 * 
 */
#include "Detector.h"

// 裝甲板排序
bool SortArmor(Armor armor1, Armor armor2)
{
    return armor1.conf > armor2.conf;
}
/**
 * @brief 初始化推理引擎
 * 
 */
void Detector::InitEngine()
{
    // 反序列化车辆引擎
    this->robotContext = DeserializeEngine(path_param.carengine_path, car_param.DEVICE);
    // 反序列化装甲板引擎
    this->armorContext = DeserializeEngine(path_param.armorengine_path, armor_param.DEVICE);
}

/**
 * @brief 核心函数
 * 
 * @param  src_img          源图像
 */
void Detector::Detect(Mat &src_img, vector<Robot_ptr> &robots)
{
    this->src = src_img;
    // 寻找机器人
    this->FindRobot(src_img, robots);
}

/**
 * @brief 构造函数
 * 
 */
Detector::Detector(bool baseFlag)
{
    // 初始化推理引擎
    this->InitEngine();
    this->baseFlag = baseFlag;
}

/**
 * @brief 寻找机器人
 * 
 * @param src 源图像
 */
void Detector::FindRobot(Mat &src, vector<Robot_ptr> &robots)
{
    vector<Object> dectRes; // 检测器
    // 检测機器人
    DetectTarget(src, this->robotContext, dectRes, 0);
    vector<DetectBox> detectBoxes;
    for (auto dBox : dectRes)
    {
        DetectBox detectBox;
        detectBox.id = dBox.label;
        detectBox.unblockProb = dBox.prob;
        detectBox.box = dBox.rect;
        detectBoxes.emplace_back(detectBox);
    }
    // 进行追踪
    int result = TargetSort(detectBoxes, this->trackers);
    // 進行分類處理
    vector<Armor> armors;
    for (int i = 0; i < trackers.size(); i++)
    {
        auto tracker = trackers[i];
        // 如果不允許輸出，則跳過
        if (!tracker.outputFlag)
        {
            continue;
        }
        // 如果需要重置
        if (tracker.resetFlag)
        {
            // 获取预测结果
            Rect res = tracker.tracker.get_state();
            if (res.x == 0 && res.y == 0 && res.height == 0 && res.height == 0)
            {
                continue;
            }
            // 處理越界問題
            if (res.y > src.size().height || res.x > src.size().width)
            {
                continue;
            }
            int top = max(res.y, 0);
            int bottom = min(res.y + res.height, src.size().height);
            int left = max(res.x, 0);
            int right = min(res.x + res.width, src.size().width);
            //检测器
            vector<Object> classifiRes;
            Mat ROI = src(Range(top, bottom), Range(left, right));
            // 检测裝甲板
            DetectTarget(ROI, this->armorContext, classifiRes, 1);
            if (tracker.id != -1 && tracker.id < 10)
            {
                trackers[i].armorProb = 0;
                robots[tracker.id]->setArmorConf(0);
            }
            // 將裝甲板存儲起來
            for (auto classifi : classifiRes)
            {
                Armor armor;
                armor.armorId = classifi.label;
                armor.conf = classifi.prob;
                armor.trackerId = i;
                armors.emplace_back(armor);
            }
        }
        else
        {
            // 如果不需要重置，只需要設置一下位置即可
            int robotId = tracker.id;
            robots[robotId]->setFindFlag(true);
            robots[robotId]->setBaseFlag(this->baseFlag);
            robots[robotId]->setUnBlockProb(tracker.unblockProb);
            robots[robotId]->setImgLocation((Rect)tracker.tracker.get_state());
        }
    }
    // 根據裝甲板的conf進行降序排列
    sort(armors.begin(), armors.end(), SortArmor);
    // 對分類好的數據進行處理
    for (auto armor : armors)
    {
        float oldClassifiConf = trackers[armor.trackerId].armorProb;
        float newClassifiConf = armor.conf;
        // 如果新分類出來的結果不如老分類結果，說明不對，跳過
        if (newClassifiConf < oldClassifiConf)
        {
            continue;
        }
        int oldRobotId = trackers[armor.trackerId].id;
        int newRobotId = armor.armorId;

        if (newRobotId >= 10)
        {
            continue;
        }
        // 如果跟蹤的這個大於現有的概率，再處理Robot中的數據
        if (robots[newRobotId]->getFindFlag() && newClassifiConf < robots[newRobotId]->getArmorProb())
        {
            continue;
        }

        // 先修改跟蹤器中的內容
        if (newRobotId == oldRobotId && oldRobotId != -1)
        {
            // 如果分類結果與上次相同且不爲0，則相同分類次數增加
            trackers[armor.trackerId].classfiCount++;
            // 如果相同分類次數超過閾值則提權
            if (trackers[armor.trackerId].classfiCount >= sort_param.MIN_CLASSIFICATION &&
                trackers[armor.trackerId].resetRank < 2)
            {
                trackers[armor.trackerId].resetRank++;
                trackers[armor.trackerId].classfiCount = 0;
            }
        }
        // 賦值
        trackers[armor.trackerId].id = newRobotId;
        trackers[armor.trackerId].armorProb = newClassifiConf;
        trackers[armor.trackerId].resetFlag = false;

        robots[newRobotId]->setFindFlag(true);
        robots[newRobotId]->setBaseFlag(this->baseFlag);
        robots[newRobotId]->setArmorConf(newClassifiConf);
        robots[newRobotId]->setUnBlockProb(trackers[armor.trackerId].unblockProb);
        robots[newRobotId]->setImgLocation((Rect)trackers[armor.trackerId].tracker.get_state());
    }
}

/**
 * @brief 显示函数
 * 
 * @param  is_lidar         是否使用激光雷达
 */
void Detector::Draw(vector<Robot_ptr> &robots)
{
    //克隆原图
    this->srcDraw = this->src.clone();

    for (auto robot : robots)
    {
        if (!(*robot).getFindFlag() || robot->getBaseFlag()!=this->baseFlag)
        {
            continue;
        }
        int x = (*robot).getImgLoaction().x;
        int y = (*robot).getImgLoaction().y + 1;

        //下面为绘制地图要素
        char text[256];
        if ((*robot).getID() >= 0)
        {
            sprintf(text, "%s %.1f%% %.1f%%", class_names[(*robot).getID()], (*robot).getUnBlockProb() * 100, (*robot).getArmorProb() * 100);
            //sprintf(text, "%s ", class_names[(*robot).getID()]);
            cv::putText(this->srcDraw, text, cv::Point(x, y),
                        cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(165, 255, 189), 3);
        }
        //这里为画框框处理
        if ((*robot).getID() == 5) //对红方英雄的处理
        {
            rectangle(this->srcDraw, (*robot).getImgLoaction(), Scalar(0, 165, 255), 10);
        }
        else if ((*robot).getID() == 0) //对蓝方英雄
        {
            rectangle(this->srcDraw, (*robot).getImgLoaction(), Scalar(255, 165, 0), 10);
        }
        else if ((*robot).getID() == -1) //对其他处理
        {
            rectangle(this->srcDraw, (*robot).getImgLoaction(), Scalar(0, 114, 189), 3);
        }
        else if ((*robot).getID() / 5 == 1) //对红色处理
        {
            rectangle(this->srcDraw, (*robot).getImgLoaction(), Scalar(0, 0, 255), 3);
        }
        else if ((*robot).getID() / 5 == 0) //对蓝色处理
        {
            rectangle(this->srcDraw, (*robot).getImgLoaction(), Scalar(255, 0, 0), 3);
        }
        else if ((*robot).getID() / 5 == 2) //对灰色处理
        {
            rectangle(this->srcDraw, (*robot).getImgLoaction(), Scalar(96, 96, 96), 3);
        }
    }
}
