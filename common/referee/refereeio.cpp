#include "refereeio.h"

/**
 * @description: 车间通信IO口，发的一个是地面兵种通用包，一个是各机器人专属包，
 * 包内容在SerialPort/include/DataStruct.h统一定义
 * @param {uint8_t} Targetid
 * @return {*}
 */
void refereeio::writeToReferee()
{
    //赋予初始值
    PackToGroundRobotStrategyDef OriginalData;
    OriginalData.protect = 0x00;
    OriginalData.attack = 0x00;
    OriginalData.event = 0x00;
    OriginalData.suggestion = 0x00;
    this->_PackToHero.PackToGroundRobotStrat = OriginalData;
    this->_PackToEngineer.PackToGroundRobotStrat = OriginalData;
    this->_PackToInfantry.PackToGroundRobotStrat = OriginalData;
    /*
    //发送安排
    //步兵飞坡
    if (gameinfo.timeflags->fly_flag)
    {
        this->_PackToHero.PackToGroundRobotStrat.suggestion |= 0x01;
    }
    //打符提醒
    if (gameinfo.timeflags->small_buff_can_flag || gameinfo.timeflags->big_buff_can_flag)
    {
        this->_PackToInfantry.PackToGroundRobotStrat.suggestion |= 0x01;
    }
    //取矿建议
    if (gameinfo.timeflags->mine_drop_flag)
    {
        this->_PackToEngineer.PackToGroundRobotStrat.suggestion |= (0x01 << 1);
    }
    //血量处理
    //机器人血量
    for (int i = 0; i < gameinfo.weak_list.size(); i++)
    {
        //残血是我们自己人的话，保护
        int id = gameinfo.weak_list[i];
        if (id / 5 == gameinfo.game_side)
        {
            if (id / 5)
            {
                id -= 5;
            }
            uint8_t protect = (1 << id);
            this->_PackToHero.PackToGroundRobotStrat.protect |= protect;
            this->_PackToEngineer.PackToGroundRobotStrat.protect |= protect;
            this->_PackToInfantry.PackToGroundRobotStrat.protect |= protect;
        }
        else
        { //残血是敌人的话，追杀
            if (id / 5)
            {
                id -= 5;
            }
            uint8_t attack = (1 << id);
            this->_PackToHero.PackToGroundRobotStrat.protect |= attack;
            this->_PackToEngineer.PackToGroundRobotStrat.protect |= attack;
            this->_PackToInfantry.PackToGroundRobotStrat.protect |= attack;
        }
    }
    //前哨站与哨兵血量
    int my_outpost;
    int enemy_outpost;
    int my_watcher;
    int enemy_watcher;
    if (gameinfo.game_side)
    {
        //如果我方是红方
        enemy_outpost = gameinfo.HP_list.b_outpost_HP;
        my_outpost = gameinfo.HP_list.r_outpost_HP;
        enemy_watcher = gameinfo.HP_list.b_sentry_HP;
        my_watcher = gameinfo.HP_list.r_sentry_HP;
    }
    else
    {
        //如果我方是蓝方
        enemy_outpost = gameinfo.HP_list.r_outpost_HP;
        my_outpost = gameinfo.HP_list.b_outpost_HP;
        enemy_watcher = gameinfo.HP_list.r_sentry_HP;
        my_watcher = gameinfo.HP_list.b_sentry_HP;
    }
    //我方前哨站中血量
    if (my_outpost <= icon_param.outpost_middle_HP)
    {
        if (my_outpost <= icon_param.outpost_low_HP)
        {
            this->_PackToHero.PackToGroundRobotStrat.event |= 0x01;
            this->_PackToEngineer.PackToGroundRobotStrat.event |= 0x01;
            this->_PackToInfantry.PackToGroundRobotStrat.event |= 0x01;
        }
        this->_PackToEngineer.PackToGroundRobotStrat.suggestion |= 0x01;
    }
    //敌方前哨站中血量
    if (enemy_outpost <= icon_param.outpost_middle_HP)
    {
        this->_PackToHero.PackToGroundRobotStrat.suggestion |= (0x01 << 1);
    }
    //我方哨兵低血量
    if (my_watcher <= icon_param.watcher_low_HP)
    {
        this->_PackToHero.PackToGroundRobotStrat.event |= (0x01 << 1);
        this->_PackToEngineer.PackToGroundRobotStrat.event |= (0x01 << 1);
        this->_PackToInfantry.PackToGroundRobotStrat.event |= (0x01 << 1);
    }
    //敌方哨兵中血量
    if (enemy_watcher <= icon_param.watcher_middle_HP)
    {
        this->_PackToInfantry.PackToGroundRobotStrat.suggestion |= (0x01 << 1);
    }
    //抢血注意
    if (gameinfo.game_stage == 3)
    {
        this->_PackToInfantry.PackToGroundRobotStrat.suggestion |= (0x01 << 3);
        this->_PackToHero.PackToGroundRobotStrat.suggestion |= (0x01 << 2);
    }
    //英雄
    this->referee.CV_ToOtherRobot(referee.robot_client_ID.hero, this->_PackToHero.UsartData, sizeof(this->_PackToHero.UsartData));
    //步兵
    this->referee.CV_ToOtherRobot(referee.robot_client_ID.infantry_3, this->_PackToInfantry.UsartData, sizeof(this->_PackToInfantry.UsartData));
    this->referee.CV_ToOtherRobot(referee.robot_client_ID.infantry_4, this->_PackToInfantry.UsartData, sizeof(this->_PackToInfantry.UsartData));
    this->referee.CV_ToOtherRobot(referee.robot_client_ID.infantry_5, this->_PackToInfantry.UsartData, sizeof(this->_PackToInfantry.UsartData));
    //工程
    this->referee.CV_ToOtherRobot(referee.robot_client_ID.engineer, this->_PackToEngineer.UsartData, sizeof(this->_PackToEngineer.UsartData));
    */
};

//发包测试
void refereeio::testTransmit()
{

    this->referee.Radar_dataTransmit(102, (float)(12.1), (float)(12.1), .0f);
    this->referee.Radar_dataTransmit(2, (float)(12.1), (float)(12.8), .0f);
    cout<<(int)this->referee.robot_client_ID.local<<endl;

    /*
    PackToGroundRobotStrategyDef OriginalData;
    //保护英雄和步兵3，进攻步兵4和工程
    OriginalData.protect = 0x05;
    OriginalData.attack = 0x10;
    //前哨站和哨兵低血量
    OriginalData.event = 0x03;
    OriginalData.suggestion = 0x00;
    this->_PackToHero.PackToGroundRobotStrat = OriginalData;
    this->_PackToEngineer.PackToGroundRobotStrat = OriginalData;
    this->_PackToInfantry.PackToGroundRobotStrat = OriginalData;

    //英雄是注意前哨进攻和抢血
    this->_PackToHero.PackToGroundRobotStrat.suggestion |= (0x01 << 1);
    this->_PackToHero.PackToGroundRobotStrat.suggestion |= (0x01 << 2);
    //步兵是打符防守高地或者打哨兵和抢血
    this->_PackToInfantry.PackToGroundRobotStrat.suggestion |= 0x01;
    this->_PackToInfantry.PackToGroundRobotStrat.suggestion |= (0x01 << 2);
    //this->_PackToInfantry.PackToGroundRobotStrat.suggestion|= (0x01 << 1);
    //this->_PackToInfantry.PackToGroundRobotStrat.suggestion|= (0x01 << 3);
    //工程是前哨站防守和去资源岛取矿
    this->_PackToEngineer.PackToGroundRobotStrat.suggestion |= 0x01;
    this->_PackToEngineer.PackToGroundRobotStrat.suggestion |= (0x01 << 1);

    //英雄
    this->referee.CV_ToOtherRobot(referee.robot_client_ID.hero, this->_PackToHero.UsartData, sizeof(this->_PackToHero.UsartData));
    //步兵
    this->referee.CV_ToOtherRobot(referee.robot_client_ID.infantry_3, this->_PackToInfantry.UsartData, sizeof(this->_PackToInfantry.UsartData));
    this->referee.CV_ToOtherRobot(referee.robot_client_ID.infantry_4, this->_PackToInfantry.UsartData, sizeof(this->_PackToInfantry.UsartData));
    this->referee.CV_ToOtherRobot(referee.robot_client_ID.infantry_5, this->_PackToInfantry.UsartData, sizeof(this->_PackToInfantry.UsartData));
    //工程
    this->referee.CV_ToOtherRobot(referee.robot_client_ID.engineer, this->_PackToEngineer.UsartData, sizeof(this->_PackToEngineer.UsartData));
    */
}

/**
 * @description: 发送绘制小地图专属包IO口，包括颜色坐标编号，只能显示对方的坐标
 * @param {vector<Point>} &point_map, vector<int> &id
 * @param {int} side
 * @return {*}
 */
void refereeio::sendMapData(vector<Point> &point_map, vector<int> &id)
{
    uint8_t robot_id[] = {101, 102, 103, 104, 105, 1, 2, 3, 4, 5}; //id序列

    for (int i = 0; i < id.size(); i++)
    {
        if (id[i] / 5 == gameinfo.game_side)
        {
            //自己的不发
            continue;
        }
        cout<<"our side:"<<gameinfo.game_side<<endl;
        // red is true; blue is false
        if (false)
        {
            this->referee.Radar_dataTransmit(robot_id[id[i]], (float)(-((((point_map[i].y) / 684.f) * 28.f) - 28.f)), (float)(15.f - (((point_map[i].x) / 369.f) * 15.f)), .0f);
            //左下原点，m为单位
        }
        else{
            this->referee.Radar_dataTransmit(robot_id[id[i]], (float)((point_map[i].y / 684.f) * 28.f), (float)((point_map[i].x / 369.f) * 15.f), .0f);
            //左下原点，m为单位
        }

    }
};

/**
 * @brief 读取裁判系统数据并写入黑板
 * 
 * @param  judge            裁判系统
 */
bool writeBlackBoard(shared_ptr<refereeio> judge)
{
    //上次串口读取的比赛状态
    static int last_status = 0;
    //读取裁判系统信息
    if (judge->referee._SerialPort.isOpen() == false)
    {
        return false;
    }
    if (judge->readRefereeData() == false)
    {
        cout << "读取失败" << endl;
        return false;
    }
    //读取比赛剩下的时间
    gameinfo.judge_time = judge->referee.GameState.stage_remain_time;
    //读取飞镖仓门时间
    int now_dart_time = judge->referee.DartRemainTime.dart_remaining_time;
    if (now_dart_time != 0 && !gameinfo.timeflags->dart_open_flag)
    {
        gameinfo.timeflags->dart_open_flag = true;      // 仓门开启标志位为真
        gameinfo.dart_open_time = gameinfo.remain_time; // 记录仓门开启时间
    }
    if (gameinfo.timeflags->dart_open_flag)
    {
        if ((gameinfo.dart_open_time - gameinfo.remain_time) > 20)
        {
            gameinfo.timeflags->dart_open_flag = false; // 仓门开启标志位为假
        }
    }

    //读取比赛状态
    if (judge->referee.GameState.game_progress != 4)
    {
        gameinfo.game_status = 0;
    }
    else
    {
        gameinfo.game_status = 1;
    }
    //当比赛状态发生切换时的操作
    if (last_status != gameinfo.game_status)
    {
        ResetFlags();
        if (gameinfo.game_status == 1)
        {
            //开始比赛则开始倒计时
            cout << "比赛开始" << endl;
            gameinfo.timeflags->mine_drop_flag = true;
        }
    }
    last_status = gameinfo.game_status;
    //场地事件读取
    uint32_t event_type = judge->referee.EventData.event_type;
    //读取护盾状态
    if ((event_type >> 22) & 1 == 0)
    {
        gameinfo.timeflags->virtual_shield_flag = false;
    }
    //读取能量机关状态
    if (((event_type >> 26) & 1 || (event_type >> 27) & 1) == 1)
    {
        gameinfo.timeflags->in_buff_flag = true;
        //cout << "裁判系统能量机关标志位激活" << endl;
    }
    else
    {
        gameinfo.timeflags->in_buff_flag = false;
        //cout << "能量机关不激活" << endl;
    }

    //机器人血量读取
    //红方
    gameinfo.HP_list.r_outpost_HP = judge->referee.GameRobotHP.red_outpost_HP;
    gameinfo.HP_list.r_sentry_HP = judge->referee.GameRobotHP.red_7_robot_HP;
    gameinfo.HP_list.r_base_HP = judge->referee.GameRobotHP.red_base_HP;
    gameinfo.HP_list.R1 = judge->referee.GameRobotHP.red_1_robot_HP;
    gameinfo.HP_list.R2 = judge->referee.GameRobotHP.red_2_robot_HP;
    gameinfo.HP_list.R3 = judge->referee.GameRobotHP.red_3_robot_HP;
    gameinfo.HP_list.R4 = judge->referee.GameRobotHP.red_4_robot_HP;
    gameinfo.HP_list.R5 = judge->referee.GameRobotHP.red_5_robot_HP;
    //蓝方
    gameinfo.HP_list.b_outpost_HP = judge->referee.GameRobotHP.blue_outpost_HP;
    gameinfo.HP_list.b_sentry_HP = judge->referee.GameRobotHP.blue_7_robot_HP;
    gameinfo.HP_list.b_base_HP = judge->referee.GameRobotHP.blue_base_HP;
    gameinfo.HP_list.B1 = judge->referee.GameRobotHP.blue_1_robot_HP;
    gameinfo.HP_list.B2 = judge->referee.GameRobotHP.blue_2_robot_HP;
    gameinfo.HP_list.B3 = judge->referee.GameRobotHP.blue_3_robot_HP;
    gameinfo.HP_list.B4 = judge->referee.GameRobotHP.blue_4_robot_HP;
    gameinfo.HP_list.B5 = judge->referee.GameRobotHP.blue_5_robot_HP;
    return true;
}
