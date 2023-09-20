/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file   : referee.cpp
  * @author : Lingzi_Xie 1357657340@qq.com
  * @brief  : Code for communicating with Referee system of Robomaster 2021.
  * @date   : 2021-03-21
  * @par Change Log：
  *  Date           Author   Version    Notes
  *  2019-03-01     charlie   2.0.0     Creator
  *  2019-12-28     kainan	  3.0.0     增加绘画类
  *  2020-05-26 	kainan    4.0.0		适应20年规则
  *  2021-03-21		Lingzi    5.0.0		接收数据包格式适应21年高校联盟赛规则 	
  *  2021-05-19		Lingzi    5.4.1		优化了实现中的一些冗余变量和代码
  *  2021-05-20		Lingzi    5.4.2		适配21年超级对抗赛规则
  *  2021-05-22		Lingzi    5.4.5		添加工程、哨兵、无人机的完整UI
  *  
  ==============================================================================
                          How to use this driver  
  ==============================================================================
	Init()初始化模块
	
	裁判系统数据接收与外部读取
	1.使用unPackDataFromRF()解包裁判系统串口数据
	2.如果需要用到裁判系统提供的各种数据（具体有些什么数据请查看手册），读取相应结构体即可
	
	机器人车间通信
	1. 发送端调用CV_ToOtherRobot()发送数据
	2. 接收端轮询机器人ID对应的robot_rec_data[]数据情况，如工程发送过来的数据为robot_rec_data[1]

	操作手界面UI
	1.Set_DrawingLayer()设置图层，0-9
	2.对应的drawin()，clean()
	3.组合图形:标尺UI_ruler()、Draw_FitingGraph()
	4.注意：发数据给裁判系统。务必注意等待上电稳定之后才发送，否则会丢包 \n
					每个图形的name必须不同，建议name[3]不同即可 \n
	
	注意：
	串口DMA接收缓存数组大小建议设置等于256，过小容易造成解包过程数据被覆盖。发送需要和串口接收不同任务，由于速率控制内部有大量的vTaskDelay
	要等待一段时间(等串口、裁判系统稳定)，再发送clean、数据、UI等 
	特别注意要用最新的裁判系统客户端，旧版有点问题
	
	
  	目前参考的是裁判系统串口协议附录V1.0-2021-02
  	如有问题，请参考《RM2021裁判系统用户接口协议附录V1.0》

  ******************************************************************************
  * @attention:
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version NO., write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "referee.h"

/* Private define ------------------------------------------------------------*/
/* Private function declarations --------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* 8位CRC校验码，用于数据帧帧头校验 */
static const unsigned char CRC8_TAB[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3,
    0xfd, 0x1f, 0x41, 0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01,
    0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe,
    0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 0xbe, 0xe0, 0x02, 0x5c,
    0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 0x46,
    0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb,
    0x59, 0x07, 0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5,
    0xfb, 0x78, 0x26, 0xc4, 0x9a, 0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
    0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99,
    0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 0x8c, 0xd2,
    0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93,
    0xcd, 0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31,
    0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d,
    0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d,
    0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 0xca, 0x94, 0x76,
    0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4,
    0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75,
    0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9,
    0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

/* 16位CRC校验码，用于数据帧整帧校验 */
static const uint16_t wCRC_Table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e,
    0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64,
    0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e,
    0xfae7, 0xc87c, 0xd9f5, 0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
    0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 0xce4c, 0xdfc5, 0xed5e,
    0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44,
    0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e,
    0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e,
    0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff, 0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324,
    0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e,
    0xf2a7, 0xc03c, 0xd1b5, 0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
    0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 0xc60c, 0xd785, 0xe51e,
    0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704,
    0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e,
    0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1,
    0x0f78
};
/* Private type --------------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 
 * @note Template of data len limitation, use in SendDrawData() and character_drawing()
 * @param 
 * @retval 
 */
template<typename Type>
Type _referee_Constrain(Type input,Type min,Type max){
  if (input <= min)
    return min;
  else if(input >= max)
    return max;
  else return input;
}

/**
  * @brief   CRC8 data check.
  * @param   *pchMessage:Data to be processed
              dwLength:Length of check data
              ucCRC8:Data after processing
  * @retval  	Gets the CRC8 	
  */
unsigned char referee_Classdef::Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8)
{
    unsigned char ucIndex;
    while (dwLength--) {
        ucIndex = ucCRC8^(*pchMessage++);										
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return(ucCRC8);
}
/**
  * @brief   CRC16 data check.
  * @param   *pchMessage:Data to be processed
             dwLength:Length of check data
             ucCRC8:Data after processing
  * @retval  Gets the CRC16 checksum
  */
uint16_t referee_Classdef::Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == NULL) {
        return 0xFFFF;
    }
    while(dwLength--) {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}


/* ----------------------------------对接收数据帧进行解包--------------------------------------- */

/**
  * @brief   学生串口数据帧解包
  * @param   *data_buff: 指向串口接收到的数据帧
			 *length：接收到的数据帧长度

* @retval    对学生串口数据帧进行解包，并按照数据包的类型将信息存储到对应的结构体中
  */
void referee_Classdef::unPackDataFromRF(uint8_t *data_buf, uint32_t length)
{
	static uint8_t RFdataBuff[256];														//将data_buf指向的数据帧复制到缓存区中

	static int32_t index,buff_read_index;												//解包结果存储区索引，及缓存区索引								
	static short CRC16_Function,CRC16_Referee;											//记录整帧的CRC16计算结果，及帧末的CRC16数值
	static uint8_t byte;
	static int32_t read_len;															//记录数据帧总体字节数
	static uint16_t data_len;															//记录数据帧的数据段字节数
	static uint8_t unpack_step;															//记录当前数据帧的解包状态
	static uint8_t protocol_packet[PROTOCAL_FRAME_MAX_SIZE];							//解包结果存储区
	
    /*初始化读取状态*/
    buff_read_index = 0;
    memcpy(RFdataBuff,data_buf,length);
	
    /*从数据帧帧头开始读取 */
    read_len=length;
	
    while (read_len--) 
	{
		byte = RFdataBuff[buff_read_index++];											//载入缓存区当前数据
		switch(unpack_step) {
		case STEP_HEADER_SOF: {
			if(byte == START_ID) {
				unpack_step = STEP_LENGTH_LOW;											//将数据帧帧头的SOF写入结果存储区
				protocol_packet[index++] = byte;										
			} else {
				index = 0;
			}
		}
		break;

        case STEP_LENGTH_LOW: {
            data_len = byte;
            protocol_packet[index++] = byte;											//将数据帧帧头的len低八位写入结果存储区										
            unpack_step = STEP_LENGTH_HIGH;
        }
        break;

        case STEP_LENGTH_HIGH: {
            data_len |= (byte << 8);
            protocol_packet[index++] = byte;
            if(data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CRC_ALL_LEN)) {	
                unpack_step = STEP_FRAME_SEQ;											//将数据帧帧头的len高八位写入结果存储区
            } else {
                unpack_step = STEP_HEADER_SOF;											//若数据帧过长，则放弃接收该帧
                index = 0;
            }
        }
        break;

        case STEP_FRAME_SEQ: {	
            protocol_packet[index++] = byte;											//将数据帧帧头的包序列SEQ写入结果存储区						
            unpack_step = STEP_HEADER_CRC8;
        }
        break;

        case STEP_HEADER_CRC8: {														//帧头CRC8查表校验
            protocol_packet[index++] = byte;											//将CRC8校验码写入结果缓存区

            if (index == HEADER_LEN+1) {
                if ( Get_CRC8_Check_Sum(protocol_packet, HEADER_LEN,0xff)== protocol_packet[HEADER_LEN]) {
                    unpack_step = STEP_DATA_CRC16;										//帧头校验成功，则进入整帧校验
                } else {
                    unpack_step = STEP_HEADER_SOF;
                    index = 0;
                }
            }
        }
        break;

        case STEP_DATA_CRC16: {
            if (index < (HEADER_LEN + CMD_LEN + data_len + CRC_ALL_LEN)) {				//未访问完整帧字节
                protocol_packet[index++] = byte;
            }
            if (index >= (HEADER_LEN + CMD_LEN + data_len + CRC_ALL_LEN)) {				//访问完整帧，接下来开始CRC16校验

                CRC16_Function=Get_CRC16_Check_Sum(protocol_packet, HEADER_LEN + CMD_LEN + data_len +CRC_8_LEN,0xffff);
                CRC16_Referee=* (short *)(&protocol_packet[index-2]);			//取出数据帧帧末的CRC16校验和
                if ( CRC16_Function==CRC16_Referee) {									//对比CRC校验结果
                    RefereeHandle(protocol_packet);										//校验完成，将解包结果按照cmd_id转存到类的对应成员结构体中
                }
                unpack_step = STEP_HEADER_SOF;
                index = 0;
            }
        }
        break;

        default: {																		//其他情况则直接重置解包状态和索引
            unpack_step = STEP_HEADER_SOF;
            index = 0;
        }
        break;
        }
    }
}

/**
  * @brief  Receive and handle referee system data
  * @param  *data_buf:data which is unpacked successfully
  * @retval void
  */
void referee_Classdef::RefereeHandle(uint8_t *data_buf)
{
    switch(((FrameHeader *)data_buf)->CmdID) {											//取出解包结果的cmd_id

			case GameState_ID: 
				GameState = *(ext_game_status_t*)(&data_buf[7]);						//取解包结果数据段的段起始字节地址，转化为对应结构体类型的指针，并对该指针取值，即得数据
			break;
			
			case GameResult_ID: 
					GameResult = *(ext_game_result_t*)(&data_buf[7]);
			break;
			
			case GameRobotHP_ID: 
					GameRobotHP = *(ext_game_robot_HP_t*)(&data_buf[7]);
			break;	
			
			case DartStatus_ID: 
					DartStatus = *(ext_dart_status_t*)(&data_buf[7]);
			break;			
			
			case EventData_ID: 
					EventData = *(ext_event_data_t*)(&data_buf[7]);
			break;			
			
			case SupplyProjectileAction_ID: 
					SupplyAction = *(ext_supply_projectile_action_t*)(&data_buf[7]);
			break;			
			
			case RefereeWarning_ID: 
					RefereeWarning = *(ext_referee_warning_t*)(&data_buf[7]);
			break;			
			
			case DartRemainingTime_ID: 
					DartRemainTime = *(ext_dart_remaining_time_t*)(&data_buf[7]);
			break;			
			
			case GameRobotState_ID: 
					GameRobotState = *(ext_game_robot_status_t*)(&data_buf[7]);
					Calc_Robot_ID(GameRobotState.robot_id);
			break;			
						
			case PowerHeatData_ID: 
					PowerHeatData = *(ext_power_heat_data_t*)(&data_buf[7]);
			break;			
			
			case GameRobotPos_ID: 
					RobotPos = *(ext_game_robot_pos_t*)(&data_buf[7]);
			break;			
			
			case BuffMusk_ID: 
					RobotBuff = *(ext_buff_t*)(&data_buf[7]);
			break;			
			
			case AerialRobotEnergy_ID: 
					AerialEnergy = *(aerial_robot_energy_t*)(&data_buf[7]);
			break;			
			
			case RobotHurt_ID: 
					RobotHurt = *(ext_robot_hurt_t*)(&data_buf[7]);
			break;			
			
			case ShootData_ID: 
					ShootData = *(ext_shoot_data_t*)(&data_buf[7]);
			break;			
						
			case BulletRemaining_ID: 
					BulletRemaining = *(ext_bullet_remaining_t*)(&data_buf[7]);
			break;						
			
			case RFID_Status_ID: 
					RFID_Status = *(ext_rfid_status_t*)(&data_buf[7]);
			break;			
						
			case ExtDartClientCmd_ID:				
					DartClientCmd = *(ext_dart_client_cmd_t*)(&data_buf[7]);
			break;
			
			case StudentInteractiveHeaderData_ID:																//若为数据交互的数据帧，则交给其他成员函数进行处理							
					RobotInteractiveHandle((robot_interactive_data_t*)(&data_buf[7]));							//robot_interactive_data_t是整个数据段！
			break;

			case CustomControllerData_ID:
					custom_control_data = *(custom_controller_interactive_data_t*)(&data_buf[7]);
			break;

			case MiniMapInteractiveData_ID:
					mini_map_data = *(ext_mini_map_command_t*)(&data_buf[7]);
			break;
			default:
					break;
    }
}

/**
  * @brief  处理车间通信数据包 
  * @param  
  * @retval 
  */
void referee_Classdef::RobotInteractiveHandle(robot_interactive_data_t* RobotInteractiveData_t)
{
	if(GameRobotState.robot_id == RobotInteractiveData_t->receiver_ID && GameRobotState.robot_id != 0) {		//如果确实是发往本机器人的交互数据帧，则接受该帧
		if(RobotInteractiveData_t->data_cmd_id == RobotComData_ID) {											//根据发送者的ID号归纳数据段并存储到对应区域
			if(RobotInteractiveData_t->sender_ID > 100)															//蓝方数据接收
				memcpy(&robot_rec_data[RobotInteractiveData_t->sender_ID - 101], RobotInteractiveData_t->data, ROBOT_COM_PACK);
			else																								//红方数据接收
				memcpy(&robot_rec_data[RobotInteractiveData_t->sender_ID - 1], RobotInteractiveData_t->data, ROBOT_COM_PACK);
		}
	}
}

/**
  * @brief  Calculate robot ID 
  * @param  local robot id, which come from referee
  * @retval 
  */
void referee_Classdef::Calc_Robot_ID(uint8_t local_id)
{
	uint8_t *id_ptr = (uint8_t*)&robot_client_ID;
	uint8_t i = 1;

	if(local_id !=0 )																	
	{
		if(local_id < 10)																//计算当前机器人的ID号（红方）
		{
			for(i = 1;i < 10;i++)
				(*id_ptr++) = i;

			robot_client_ID.robot_where = Robot_Red;
			robot_client_ID.local = local_id;
			robot_client_ID.client = 0x100 + local_id;									
		}
		else																			//计算当前机器人的ID号（蓝方）
		{
			for(i = 1;i < 10;i++)
				(*id_ptr++) = i + 100;

			robot_client_ID.robot_where = Robot_Blue;	
			robot_client_ID.local = local_id;	
			robot_client_ID.client = 0x0100 + local_id;							
		}
	}
}

/* ----------------------------------底层数据帧发送及速率控制---在PC端有改动--------------------------------------- */

/**
 * @brief 打包机器人之间的交互数据，以及UI数据下发到底层发送
 * @param 
 */
void referee_Classdef::pack_send_robotData(uint16_t _data_cmd_id, uint16_t _receiver_ID, uint8_t* _data, uint16_t _data_len)
{
	static uint8_t temp[112];															//官方约定机器人之间的交互数据帧的数据段不得超过113字节
	
	DataHeader data_header;																//定义数据段段首并设置										
	data_header.data_cmd_id = _data_cmd_id;
	data_header.send_ID = robot_client_ID.local;
	data_header.receiver_ID = _receiver_ID;
	
	uint8_t header_len = sizeof(data_header);											
	memcpy((void*)temp, &data_header, header_len);										//将数据帧的数据段进行封装（封装段首）
	memcpy((void*)(temp + header_len), _data, _data_len);								//将数据帧的数据段进行封装（封装数据）
	
	if(data_header.receiver_ID == robot_client_ID.client)								//若交互数据正好要送给自身的裁判系统
		send_toReferee(StudentInteractiveHeaderData_ID, temp, header_len+_data_len, UI_Client);
	else																				//交互数据送给其他机器人的裁判系统
		send_toReferee(StudentInteractiveHeaderData_ID, temp, header_len+_data_len, CV_OtherRobot);
}

/**
 * @brief 底层发送函数。负责发送数据包，以及对发送速率做控制
 * @param _cmd_id，
 * @param _data，
 * @param data_len，
 * @param _receive_type，判断车间通信 or 雷达站 or UI，决定数据需不需要发多次（客户端数据经常丢包，所以需要发多次），以及发送频率
 */
void referee_Classdef::send_toReferee(uint16_t _cmd_id,uint8_t* _data, uint16_t _data_len, receive_Type_e _receive_type)
{	
	static PackToSend Pack;									//串口发送数据包


	static uint8_t seq = 0;
																				//用于控制发送速率
	FrameHeader send_frame_header;																							//交互数据帧帧头设置	

	send_frame_header.SOF = START_ID;
	send_frame_header.DataLength = _data_len;
	send_frame_header.Seq = seq++;
	send_frame_header.CRC8 = Get_CRC8_Check_Sum((uint8_t*)&send_frame_header,4,0xff);
	send_frame_header.CmdID = _cmd_id;
	
	uint8_t header_len = sizeof(send_frame_header);
	
	memcpy((void*)Pack._temp, &send_frame_header, header_len);																	//将帧头装入缓存区
	memcpy((void*)(Pack._temp+header_len), _data, _data_len);																		//将数据段转入缓存区	
	
	* (short *)(&Pack._temp[header_len + _data_len]) = Get_CRC16_Check_Sum(Pack._temp,header_len + _data_len,0xffff);	
	
	uint8_t send_cnt = 3;																									//传输次数，多次传输时用
																		//header_len + _data_len + CRC16_len
	
	while(send_cnt != 0)
	{
		_SerialPort.writeStruct(Pack);									//延时已到，则发送
		
		if(_receive_type == UI_Client)
			send_cnt--;
		else															//客户端小地图绘制以及车间通信，只需要通信一次
			send_cnt = 0;
	}
	
}

/* ----------------------------------车间通信，雷达站通信以及UI绘制--------------------------------------- */

/**
  * @brief  己方机器人间的通信
  * @note   注意通信数据应小于113字节，且通信频率不得超过30Hz
  * @retval void
  */
void referee_Classdef::CV_ToOtherRobot(uint8_t target_id, uint8_t* _data, uint8_t length)
{
	pack_send_robotData(RobotComData_ID, target_id, (uint8_t*)_data, length);
}

/**
 * @brief 雷达站坐标信息发送，0x305
 * @brief 看官方的协议应该是把信息发送出去就会自动转发到本方客户端的小地图进行显示
 * @param 
 */
void referee_Classdef::Radar_dataTransmit(uint8_t target_id, float position_x, float position_y, float toward_angle)
{
	radar_map_data.target_robot_ID=target_id;
	radar_map_data.target_position_x = position_x;
	radar_map_data.target_position_y = position_y;
	radar_map_data.toward_angle = toward_angle;//今年的bug，以后请留心，要加一个随意的float才能正常显示

	uint8_t radar_data_len = sizeof(radar_map_data);								
	memcpy((void*)transmit_pack, &radar_map_data, radar_data_len);				//将数据帧的数据段进行封装（封装数据）
	send_toReferee(ClientMapCommand_ID, transmit_pack, radar_data_len, MiniMap_Client);
}

/**
 * @brief 空操作数据包
 * @param 
 * @retval 
 */
graphic_data_struct_t* referee_Classdef::null_drawing(uint8_t _layer, uint8_t name[])
{
	static graphic_data_struct_t drawing;
	memcpy(drawing.graphic_name,name,3);	

	drawing.operate_tpye = NULL_OPERATION;
	
	return &drawing;
}

/**
 * @brief 直线绘制数据包
 * @param line_width 线宽
 * @retval 
 */
graphic_data_struct_t* referee_Classdef::line_drawing(uint8_t _layer,drawOperate_e _operate_type,uint16_t startx,uint16_t starty,uint16_t endx,uint16_t endy, uint16_t line_width, colorType_e vcolor,uint8_t name[])
{
	static graphic_data_struct_t drawing;
	
	memcpy(drawing.graphic_name,name,3);																			//图案名称，3位
	drawing.layer = _layer;
	drawing.operate_tpye = _operate_type;
	drawing.graphic_tpye = LINE;
	drawing.width = line_width;
	drawing.color = vcolor;
	drawing.start_x=startx;
	drawing.start_y=starty;
	drawing.end_x=endx;
	drawing.end_y=endy;
	
	return &drawing;
}

/**
 * @brief 矩形绘制
 * @note 
 * @param line_width 线宽
 * @retval 
 */
graphic_data_struct_t* referee_Classdef::rectangle_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t startx,uint16_t starty,uint16_t length,uint16_t width, uint16_t line_width, colorType_e vcolor, uint8_t name[])
{
	static graphic_data_struct_t drawing;
	
	memcpy(drawing.graphic_name, name, 3);
	drawing.layer = _layer;	
	drawing.operate_tpye = _operate_type;
	drawing.graphic_tpye = RECTANGLE;
	drawing.width = line_width;
	drawing.color = vcolor;
	drawing.start_x = startx;
	drawing.start_y = starty;
	drawing.end_x = startx+length;
	drawing.end_y = starty+width;
	
	return &drawing;
}

/**
 * @brief 圆圈绘制
 * @note 
 * @param 
 * @retval 
 */
graphic_data_struct_t* referee_Classdef::circle_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t centrex,uint16_t centrey,uint16_t r, uint16_t line_width, colorType_e vcolor, uint8_t name[])
{
	static graphic_data_struct_t drawing;
	
	memcpy(drawing.graphic_name, name, 3);	
	drawing.layer = _layer;	
	drawing.operate_tpye = _operate_type;
	drawing.graphic_tpye = CIRCLE;
	drawing.width = line_width;
	drawing.color=vcolor;
	drawing.start_x=centrex;
	drawing.start_y=centrey;
	drawing.radius = r;
	
	return &drawing;
}

/**
 * @brief 椭圆绘制
 * @note 
 * @param minor_semi_axis x轴长
 * @param major_semi_axis y轴长
 * @retval 
 */
graphic_data_struct_t* referee_Classdef::oval_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t centrex,uint16_t centrey,uint16_t minor_semi_axis,uint16_t major_semi_axis, uint16_t line_width, colorType_e vcolor, uint8_t name[])
{
	static graphic_data_struct_t drawing;
	
	memcpy(drawing.graphic_name, name, 3);	
	drawing.layer = _layer;
	drawing.operate_tpye = _operate_type;	
	drawing.graphic_tpye=OVAL ;
	drawing.width=line_width;
	drawing.color=vcolor;
	drawing.start_x=centrex;
	drawing.start_y=centrey;
	drawing.end_x=major_semi_axis;
	drawing.end_y=minor_semi_axis;	
	
	return &drawing;
}

/**
 * @brief 椭圆弧绘制
 * @note 
 * @param 
 * @retval 
 */
graphic_data_struct_t* referee_Classdef::arc_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t centrex,uint16_t centrey,uint16_t minor_semi_axis,uint16_t major_semi_axis,int16_t start_angle,int16_t end_angle, uint16_t line_width, colorType_e vcolor, uint8_t name[])
{
	static graphic_data_struct_t drawing;
	
	memcpy(drawing.graphic_name, name, 3);	
	drawing.layer = _layer;
	drawing.operate_tpye = _operate_type;	
	drawing.graphic_tpye=ARC ;
	drawing.width=line_width;
	drawing.color=vcolor;
	drawing.start_x=centrex;
	drawing.start_y=centrey;
	drawing.end_x=minor_semi_axis;
	drawing.end_y=major_semi_axis;
	drawing.start_angle=start_angle;
	drawing.end_angle=end_angle;
	
	return &drawing;
}

/**
 * @brief 浮点数绘制【新版客户端暂时不能用，坐等官方更新】
 * @note 
 * @param 
 * @retval 
 */
graphic_data_struct_t* referee_Classdef::float_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t startx,uint16_t starty, uint16_t size, uint16_t width, colorType_e vcolor, float data, uint8_t name[])
{
	static graphic_data_struct_t drawing;
	static uint8_t* drawing_ptr = (uint8_t* )&drawing;
	
	memcpy(drawing.graphic_name, name, 3);	
	drawing.layer = _layer;
	drawing.operate_tpye = _operate_type;	
	drawing.graphic_tpye=5U ;
	drawing.start_angle = size;																						//这里是设置字体大小，需要调试
	drawing.end_angle = 3;																							//这里是设置有效小数位数，需要调试
	drawing.width=width;
	drawing.color=vcolor;
	drawing.start_x=startx;
	drawing.start_y=starty;
	
	memcpy((void*)(drawing_ptr + 11), (uint8_t*)&data, 4);															//将32位浮点数赋值到drawing结构体中
	return &drawing;
}

/**
 * @brief 整数绘制【新版客户端暂时不能用，坐等官方更新】
 * @note 
 * @param 
 * @retval 
 */
graphic_data_struct_t* referee_Classdef::int_drawing(uint8_t _layer, drawOperate_e _operate_type, uint16_t startx,uint16_t starty,uint16_t size, uint16_t width, colorType_e vcolor, int32_t data,uint8_t name[])
{
	static graphic_data_struct_t drawing;
	static uint8_t* drawing_ptr = (uint8_t* )&drawing;
	
	memcpy(drawing.graphic_name, name, 3);	
	drawing.layer = _layer;
	drawing.operate_tpye = _operate_type;	
	drawing.graphic_tpye=_INT ;
	drawing.start_angle = size;																						//这里是设置字体大小，需要调试
	drawing.width = width;
	drawing.color=vcolor;
	drawing.start_x=startx;
	drawing.start_y=starty;
	
	memcpy((void*)(drawing_ptr + 11), (uint8_t*)&data, 4);															//将32位整数赋值到drawing结构体中
	return &drawing;
}

/**
 * @brief 字符串绘制
 * @note 
 * @param 
 * @retval 
 */
graphic_data_struct_t* referee_Classdef::character_drawing(uint8_t _layer, drawOperate_e _operate_type, uint16_t startx,uint16_t starty,uint16_t size, uint8_t width,uint8_t* data, uint16_t str_len, colorType_e vcolor, uint8_t name[])
{
	static graphic_data_struct_t drawing;
	static uint8_t char_length;
	
	char_length = _referee_Constrain((uint8_t)str_len, (uint8_t)0, (uint8_t)30);											//将字符串长度约束在30个之内
	
	memcpy(drawing.graphic_name, name, 3);
	drawing.layer = _layer;	
	drawing.operate_tpye = _operate_type;	
	drawing.graphic_tpye=_CHAR ;
	drawing.width=width;
	drawing.color=vcolor;
	drawing.start_x=startx;
	drawing.start_y=starty;
	drawing.radius=0;
	drawing.start_angle = size;																						//设置字符大小，推荐字体大小与线宽的比例为10:1
	drawing.end_angle=char_length;																					//设置字符串长度
	
	return &drawing;
}

/**
 * @brief 清除某个图层下的一张图片
 * @note Referee.clean_one_picture(2, test);
 * @param 
 * @retval 
 */
void referee_Classdef::clean_one_picture(uint8_t vlayer,uint8_t name[])												//删除指定图层下的指定图形
{
	static graphic_data_struct_t drawing;
	memcpy(drawing.graphic_name, name, 3);		
	drawing.layer = vlayer ;
	drawing.operate_tpye=CLEAR_ONE_PICTURE;
	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)&drawing, sizeof(drawing));
}

/**
 * @brief 清除某个图层下的两张图片
 * @note Referee.clean_two_picture(2, test1, test2);
 * @param 
 * @retval 
 */
void referee_Classdef::clean_two_picture(uint8_t vlayer,uint8_t name1[], uint8_t name2[])												//删除指定图层下的指定图形
{
	static graphic_data_struct_t drawing[2];
	memcpy(drawing[0].graphic_name, name1, 3);		
	drawing[0].layer = vlayer ;
	drawing[0].operate_tpye=CLEAR_ONE_PICTURE;

	memcpy(drawing[1].graphic_name, name2, 3);		
	drawing[1].layer = vlayer ;
	drawing[1].operate_tpye=CLEAR_ONE_PICTURE;

	pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)drawing, sizeof(drawing));
}

/**
 * @brief 清除某一个图层
 * @note Referee.clean_layer(2);
 * @param 
 * @retval 
 */
void referee_Classdef::clean_layer(uint8_t _layer)																	//删除指定图层
{	
	cleaning.layer = _layer;
	cleaning.operate_tpye = CLEAR_ONE_LAYER;
	
	pack_send_robotData(Drawing_Clean_ID, robot_client_ID.client, (uint8_t*)&cleaning, sizeof(cleaning));
}
/**
 * @brief 清除所有UI绘制图形
 * @note Referee.clean_all();
 * @param 
 * @retval 
 */
void referee_Classdef::clean_all()																					//清除所有自定义图案
{		
	cleaning.operate_tpye = CLEAR_ALL;	
	pack_send_robotData(Drawing_Clean_ID, robot_client_ID.client, (uint8_t*)&cleaning, sizeof(cleaning));
}

/* ----------------------------------裁判系统客户端UI绘制用户接口--------------------------------------- */

/**
 * @brief 【自定义图层】绘制字符串
 * @brief 字符串不得超过30个字符
 * @note 测试后的实例，对普通步兵：referee.Draw_Char(8, 1300, 700, cap_name1, cap_str, sizeof(cap_str), WHITE, ADD_PICTURE);
 */
void referee_Classdef::Draw_Char(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t* char_name, uint8_t* data, uint16_t str_len, uint16_t str_size, colorType_e _color, drawOperate_e _operate_type)
{
	for(uint8_t i = DRAWING_PACK;i < DRAWING_PACK + 30;i++)
		data_pack[i] = 0;
	
	memcpy(data_pack, (uint8_t*)character_drawing(_layer, _operate_type, start_x,start_y,str_size, str_size / 10, data, str_len, _color, char_name), DRAWING_PACK);
	memcpy(&data_pack[DRAWING_PACK], (uint8_t* )data, str_len);
	pack_send_robotData(Drawing_Char_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK + 30);
}

/**
 * @brief 【自定义图层】UI标尺绘制，一次性绘制一条标尺
 * @note  准心圆半径为24 
 * @param _sys_time, sacle_num多少条刻度线(<9),ruler_tag第几条标尺, startpoint(标尺左上角起点), step(间距),scale_long(长刻度线的长度),scale_short
 * @note 测试后的实例，对普通步兵：referee.UI_ruler(4,961,538,30,70,40,BLUE,ADD_PICTURE);
 */
uint8_t referee_Classdef::UI_ruler(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t scale_step, uint16_t scale_long, uint16_t scale_short, colorType_e _color, drawOperate_e _operate_type)
{	
	static uint8_t ruler_name[] = "ru0";				
	static uint8_t if_short = 0;
	
	ruler_name[2] = '0';
	memcpy(data_pack, (uint8_t*)circle_drawing(_layer,_operate_type,start_x,start_y,24, 3, _color, ruler_name), DRAWING_PACK);
	ruler_name[2] = '1';
	memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(_layer,_operate_type,start_x,start_y,start_x,start_y - 200, 3, _color, ruler_name), DRAWING_PACK);	
	ruler_name[2] = '2';
	memcpy(&data_pack[DRAWING_PACK*2], (uint8_t*)line_drawing(_layer,_operate_type,start_x - 100,start_y - 10,start_x + 100,start_y - 10, 3, _color, ruler_name), DRAWING_PACK);	

	for(uint8_t i = 0;i < 4;i++)
	{
		if(if_short == 0)
		{
			if_short = 1;
			ruler_name[2] = '3'+i;
			memcpy(&data_pack[DRAWING_PACK*(i + 3)], (uint8_t*)line_drawing(_layer,_operate_type,start_x - scale_short/2,start_y - 24 - scale_step*(i + 1),start_x + scale_short/2,start_y - 24 - scale_step*(i + 1), 3, _color, ruler_name), DRAWING_PACK);	
		}
		else
		{
			if_short = 0;
			ruler_name[2] = '3'+i;
			memcpy(&data_pack[DRAWING_PACK*(i + 3)], (uint8_t*)line_drawing(_layer,_operate_type,start_x - scale_long/2,start_y - 24 - scale_step*(i + 1),start_x + scale_long/2,start_y - 24 - scale_step*(i + 1), 3, _color, ruler_name), DRAWING_PACK);	
		}	
	}
	
	pack_send_robotData(Drawing_7_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*7);
}

/**
 * @brief 【自定义图层】UI准星绘制，一次性绘制一个准星，且准星中点为红外激光中心点
 * @note 测试后的实例，对普通步兵：referee.UI_Collimator(5, 961, 538, 25, YELLOW, ADD_PICTURE);
 * @param 
 */
void referee_Classdef::UI_Collimator(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t line_length, colorType_e _color, drawOperate_e _operate_type)
{
	static uint8_t point_name[] = "poi";
	static uint8_t line_name[] = "cl0";

	//中心点
	line_name[2] = '0';
	memcpy(data_pack, (uint8_t*)circle_drawing(_layer,_operate_type,start_x,start_y,1, 3, _color, point_name), DRAWING_PACK);
	
	//准星四方向横线
	memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(_layer,_operate_type,start_x - 5,start_y, start_x - 5 - line_length, start_y, 3,_color, line_name), DRAWING_PACK);
	line_name[2] = '1';
	memcpy(&data_pack[DRAWING_PACK*2], (uint8_t*)line_drawing(_layer,_operate_type,start_x + 5,start_y, start_x + 5 + line_length, start_y, 3,_color, line_name), DRAWING_PACK);
	line_name[2] = '2';	
	memcpy(&data_pack[DRAWING_PACK*3], (uint8_t*)line_drawing(_layer,_operate_type,start_x,start_y - 5, start_x, start_y - 5 - line_length, 3,_color, line_name), DRAWING_PACK);
	line_name[2] = '3';
	memcpy(&data_pack[DRAWING_PACK*4], (uint8_t*)line_drawing(_layer,_operate_type,start_x,start_y + 5, start_x, start_y + 5 + line_length, 3,_color, line_name), DRAWING_PACK);

	//空操作包
	line_name[2] = '4';	
	memcpy(&data_pack[DRAWING_PACK*5], (uint8_t*)null_drawing(_layer, line_name), DRAWING_PACK);
	line_name[2] = '5';
	memcpy(&data_pack[DRAWING_PACK*6], (uint8_t*)null_drawing(_layer, line_name), DRAWING_PACK);
	
	pack_send_robotData(Drawing_7_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*7);
}

/**
 * @brief 【自定义图层】英雄的UI标尺绘制
 * @note 测试后的实例，对英雄：
 */
void referee_Classdef::Hero_UI_ruler(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t* line_distance, uint16_t* line_length, colorType_e _color, drawOperate_e _operate_type)
{
	uint16_t total_distance = 0;
	static uint8_t line_name[] = "he0";

	//绘制初始准星横线
	line_name[2] = '0';
	memcpy(data_pack, (uint8_t*)line_drawing(_layer,ADD_PICTURE,start_x-line_length[0]/2, start_y, start_x+line_length[0]/2, start_y, 3,_color, line_name), DRAWING_PACK);
	total_distance += line_distance[0];

	//封装好榴弹准星小横线，第2到6条
	for(uint8_t i = 1;i < 6;i++)
	{
		line_name[2] = '0' + i;
		memcpy(&data_pack[DRAWING_PACK * i], (uint8_t*)line_drawing(_layer,ADD_PICTURE,start_x-line_length[i]/2, start_y-total_distance, start_x+line_length[i]/2, start_y-total_distance, 3,_color, line_name), DRAWING_PACK);
		
		total_distance += line_distance[i];				//计算竖直线总长度
	}
	
	//封装好竖直线
	line_name[2] = '6';
	memcpy(&data_pack[DRAWING_PACK * 6], (uint8_t*)line_drawing(_layer,ADD_PICTURE,start_x, start_y, start_x, start_y-total_distance, 3,_color, line_name), DRAWING_PACK);
	
	pack_send_robotData(Drawing_7_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*7);
}
void referee_Classdef::Draw_Strategy(int index)
{
	
	/* //刚启动下，重新绘制图案
	if(enable_cnt)
	{	
		//绘制电容余量字符串，需要多次绘制
		_operate_type = ADD_PICTURE;
		Draw_Char(1, center_x, center_y, cap_name1, cap_str, sizeof(cap_str), 20, WHITE, _operate_type);		
	}
	else
		_operate_type = MODIFY_PICTURE;
	
	//将电压值转化为字符串形式
	if(volt_proportion < 100)
	{
		cap_num_str[0] = ' ';
		cap_num_str[1] = num_char[volt_proportion / 10];
		cap_num_str[2] = num_char[volt_proportion % 10];
	}
	else
	{
		cap_num_str[0] = '1';
		cap_num_str[1] = '0';
		cap_num_str[2] = '0';
	}
	
	//绘制余量百分数
	Draw_Char(1, center_x + 20, center_y - 40, cap_name2, cap_num_str, sizeof(cap_num_str), 20, _color, _operate_type); */		
}

/**
 * @brief 【图层1】电容电压百分比绘制
 * @note 画图有问题的话可能是name重复了
 * @note 超级电容开启时，则打印提示字符串，同时修改车界线和准心颜色【该功能待完善】
 * @note 测试后的实例，对普通步兵：referee.Draw_Cap_Energy(num, 24, 10, boost_flag, enable_cnt, 520,800);
 */
void referee_Classdef::Draw_Cap_Energy(float current_volt, float max_volt, float min_volt, uint8_t enable_cnt, uint16_t center_x, uint16_t center_y)
{
	static uint8_t volt_proportion;
	static colorType_e _color;
	static drawOperate_e _operate_type;
	static uint8_t num_char[] = "0123456789";
	
	//电容标题提示字符串
	static uint8_t cap_str[] = "cap energy:";
	static uint8_t cap_name1[] = "cp1";
	
	//电容余电百分数
	static uint8_t cap_num_str[] = " 90%";
	static uint8_t cap_name2[] = "cp2";
	
	//电压最大值更新
	if(current_volt > max_volt)
		max_volt = current_volt;
		
	//当前电压比例计算
	volt_proportion = (uint8_t)((current_volt-min_volt)*(current_volt+min_volt) / ((max_volt-min_volt)*(max_volt+min_volt)) * 100.0);
	
	//电压比例阈值划分，显示不同的颜色
	if(volt_proportion > 70)
		_color = GREEN;
	else if(volt_proportion > 30)
		_color = YELLOW;
	else
		_color = PINK;
	
	//刚启动下，重新绘制图案
	if(enable_cnt)
	{	
		//绘制电容余量字符串，需要多次绘制
		_operate_type = ADD_PICTURE;
		Draw_Char(1, center_x, center_y, cap_name1, cap_str, sizeof(cap_str), 20, WHITE, _operate_type);		
	}
	else
		_operate_type = MODIFY_PICTURE;
	
	//将电压值转化为字符串形式
	if(volt_proportion < 100)
	{
		cap_num_str[0] = ' ';
		cap_num_str[1] = num_char[volt_proportion / 10];
		cap_num_str[2] = num_char[volt_proportion % 10];
	}
	else
	{
		cap_num_str[0] = '1';
		cap_num_str[1] = '0';
		cap_num_str[2] = '0';
	}
	
	//绘制余量百分数
	Draw_Char(1, center_x + 20, center_y - 40, cap_name2, cap_num_str, sizeof(cap_num_str), 20, _color, _operate_type);		
}
	
/**
 * @brief 【图层1】超级电容开启绘制，检测到上升沿或下降沿后，多次发包确保绘制成功 
 * @param 
 * @note 测试后的实例，对普通步兵：referee.Draw_Boost(boost_flag, 1500, 840, 10, PINK);
 */
void referee_Classdef::Draw_Boost(uint8_t boost_flag,  uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color)
{
	static uint8_t last_boost_flag = 0;
	static uint8_t enable_cnt[2] = {0,0};
	static uint8_t boost_name[] = "bos";
	
	if(boost_flag && (!last_boost_flag))			//信号上升沿，多次绘制标志
		enable_cnt[0] = 6;
	else if((!boost_flag) && last_boost_flag)		//信号下降沿，多次删除标志
		enable_cnt[1] = 6;
	else;
	
	if(enable_cnt[0])
	{
		enable_cnt[0]--;
		memcpy(data_pack, (uint8_t*)arc_drawing(1, ADD_PICTURE, center_x,center_y,20,20,45,315,line_width,_color, boost_name), DRAWING_PACK);
		pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);
	}
	else if(enable_cnt[1])
	{
		enable_cnt[1]--;
		clean_one_picture(1, boost_name);
	}

	last_boost_flag = boost_flag;
	return;
}

/**
 * @brief 【图层2】绘制静态小陀螺标志
 * @param spin_flag：小陀螺使能标志位
 * @note 测试后的实例，对普通步兵：referee.Draw_Spin(spin_flag, 1400, 840, 10, BLUE);
 */
void referee_Classdef::Draw_Spin(uint8_t spin_flag, uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color)
{
	static uint8_t last_spin_flag = 0;
	static uint8_t spin_name0[] = "sp0";
	static uint8_t spin_name1[] = "sp1";
	static uint8_t enable_cnt[2] = {0,0};
	
	if(spin_flag && (!last_spin_flag))				//信号上升沿，多次绘制标志
		enable_cnt[0] = 6;
	else if(last_spin_flag && (!spin_flag))			//信号下降沿，多次删除标志
		enable_cnt[1] = 6;
	else;
	
	if(enable_cnt[0])
	{
		enable_cnt[0]--;

		memcpy(data_pack, (uint8_t*)arc_drawing(2, ADD_PICTURE, center_x,center_y,20,20,180,270,line_width,_color, spin_name0), DRAWING_PACK);
		memcpy(&data_pack[DRAWING_PACK], (uint8_t*)arc_drawing(2, ADD_PICTURE, center_x,center_y,20,20,0,90,line_width,_color, spin_name1), DRAWING_PACK);
	
		pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);
	}
	
	if(enable_cnt[1])								//小陀螺结束时，清除图形	
	{
		enable_cnt[1]--;
		clean_two_picture(2, spin_name0, spin_name1);
	}		

	last_spin_flag = spin_flag;
	return;
}

/**
 * @brief 【图层3】弹仓开启绘制，检测到上升沿或下降沿后，多次发包确保绘制成功
 * @note 画图有问题的话可能是name重复了
 * @note 测试后的实例，对普通步兵：referee.Draw_Bullet(bullet_flag, 1600, 840, 8, GREEN);【线宽为3，黄色】
 */
void referee_Classdef::Draw_Bullet(uint8_t bullet_flag, uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color)
{
	static uint8_t last_bullet_flag = 0;
	static uint8_t enable_cnt[2] = {0,0};
	static uint8_t bullet_name[] = "be0";
	
	if(bullet_flag && (!last_bullet_flag))			//信号上升沿，多次绘制标志
		enable_cnt[0] = 6;
	else if((!bullet_flag) && last_bullet_flag)		//信号下降沿，多次删除标志
		enable_cnt[1] = 6;
	else;
	
	if(enable_cnt[0])			
	{
		enable_cnt[0]--;							//左侧

		bullet_name[2] = '0';
		memcpy(data_pack, (uint8_t*)line_drawing(3, ADD_PICTURE, center_x-20,center_y-20,center_x-20,center_y+20,line_width,_color, bullet_name), DRAWING_PACK);
		bullet_name[2] = '1';						//下方
		memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(3, ADD_PICTURE, center_x-20,center_y-20,center_x+20,center_y-20,line_width,_color, bullet_name), DRAWING_PACK);
		bullet_name[2] = '2';						//右侧
		memcpy(&data_pack[DRAWING_PACK*2], (uint8_t*)line_drawing(3, ADD_PICTURE, center_x+20,center_y-20,center_x+20,center_y+20,line_width,_color, bullet_name), DRAWING_PACK);
		bullet_name[2] = '3';						//弹丸
		memcpy(&data_pack[DRAWING_PACK*3], (uint8_t*)circle_drawing(3, ADD_PICTURE, center_x,center_y ,3,line_width,_color, bullet_name), DRAWING_PACK);
		
		bullet_name[2] = '4';						//空包
		memcpy(&data_pack[DRAWING_PACK*4], (uint8_t*)null_drawing(3, bullet_name), DRAWING_PACK);
		
		pack_send_robotData(Drawing_5_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*5);
	}
	else if(enable_cnt[1])
	{
		enable_cnt[1]--;
		clean_layer(3);
	}

	last_bullet_flag = bullet_flag;
	return;
}

/**
 * @brief 【图层0】绘制车界线，自定义绘制位置
 * @note 画图有问题的话可能是name重复了
 * @note 测试后的实例，对普通步兵：referee.Draw_Robot_Limit(180, 80, 961, 3, YELLOW, ADD_PICTURE);【线宽为3，黄色】
 */
void referee_Classdef::Draw_Robot_Limit(uint16_t height, uint16_t distance, uint16_t center_x, uint16_t line_width, colorType_e _color, drawOperate_e _operate_type)
{
	static uint8_t limit_name[] = "li0";

	//右侧车界线绘制
	limit_name[2] = '0';
	memcpy(data_pack, (uint8_t*)line_drawing(0, _operate_type, center_x + distance, height , center_x + distance + 200, height, line_width, _color, limit_name), DRAWING_PACK);
	limit_name[2] = '1';
	memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(0, _operate_type, center_x + distance + 200, height, center_x + distance + 360, height - 100, line_width, _color, limit_name), DRAWING_PACK);
	
	//左侧车界线绘制
	limit_name[2] = '2';
	memcpy(&data_pack[DRAWING_PACK*2], (uint8_t*)line_drawing(0, _operate_type, center_x - distance, height, center_x - distance - 200, height, line_width, _color, limit_name), DRAWING_PACK);
	limit_name[2] = '3';
	memcpy(&data_pack[DRAWING_PACK*3], (uint8_t*)line_drawing(0, _operate_type, center_x - distance - 200, height, center_x - distance - 360, height - 100, line_width, _color, limit_name), DRAWING_PACK);
	
	//空包
	limit_name[2] = '4';
	memcpy(&data_pack[DRAWING_PACK*4], (uint8_t*)null_drawing(0, limit_name), DRAWING_PACK);
		
	pack_send_robotData(Drawing_5_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*5);
}

/**
 * @brief 【图层0】绘制自瞄开启时的字符提示，自定义绘制位置
 * @note 画图有问题的话可能是name重复了
 * @note 测试后的实例，对普通步兵：
 */
void referee_Classdef::Draw_Auto_Lock(uint8_t auto_flag, uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color)
{
	static uint8_t last_auto_flag = 0;
	static uint8_t enable_cnt[2] = {0};
	static uint8_t auto_name0[] = "au0";
	static uint8_t auto_name1[] = "au1";

	if(auto_flag && (!last_auto_flag))
		enable_cnt[0] = 6;
	else if(last_auto_flag && (!auto_flag))
		enable_cnt[1] = 6;
	else;

	if(enable_cnt[0])
	{
		enable_cnt[0]--;

		memcpy(data_pack, (uint8_t*)circle_drawing(0, ADD_PICTURE, center_x,center_y ,3,line_width,_color, auto_name0), DRAWING_PACK);
		memcpy(&data_pack[DRAWING_PACK], (uint8_t*)circle_drawing(0, ADD_PICTURE, center_x,center_y ,20,line_width,_color, auto_name1), DRAWING_PACK);

		pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);
	}
	
	if(enable_cnt[1])
	{
		enable_cnt[1]--;
		clean_two_picture(0, auto_name0, auto_name1);
	}

	last_auto_flag = auto_flag;
	return;
}

/**
 * @brief 【图层4】No Bullet
 * @note 画图有问题的话可能是name重复了
 * @note 测试后的实例，对普通步兵：
 */
void referee_Classdef::Draw_No_Bullet(uint8_t no_bullet_flag, uint16_t center_x, uint16_t center_y, colorType_e _color)
{
	static uint8_t last_flag = 0;
	static uint8_t enable_cnt[2] = {0,0};
	static uint8_t no_bullet_name[] = "no0";
	static uint8_t warning_str[] = "No Bullet!";
	
	if(no_bullet_flag && (!last_flag))
		enable_cnt[0] = 6;
	else if((!no_bullet_flag) && last_flag)
		enable_cnt[1] = 6;
	
	if(enable_cnt[0])
	{
		enable_cnt[0]--;
		Draw_Char(4, center_x, center_y, no_bullet_name, warning_str, sizeof(warning_str), 20, _color, ADD_PICTURE);		
	}
	else if(enable_cnt[1])
	{
		enable_cnt[1]--;
		clean_one_picture(4, no_bullet_name);
	}

	last_flag = no_bullet_flag;
}

/**
 * @brief 【自定义图层】空中机器人UI标尺
 * @note 画图有问题的话可能是name重复了
 * @note 
 */
void referee_Classdef::Draw_Pitch_Ruler(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, uint16_t long_scale_length, uint16_t short_scale_length, colorType_e ruler_color, colorType_e current_color, colorType_e target_color)
{
	static uint8_t line_name[] = "600";					//Aerial id + tag

	uint16_t scale_step = total_length / 30;
	uint16_t scale_point[7];
	uint8_t i = 0;									//计数变量

	/* 生成每个大刻度线的垂直坐标 */
	scale_point[0] = center_y + total_length/2;		//第一条大刻度，+15度
	scale_point[6] = center_y - total_length/2;		//最后一条大刻度，-15度

	for(i = 1;i < 6;i++)							//中间大刻度
		scale_point[i] = scale_point[0] - scale_step*5*i;
	
	line_name[1] = '0';
	line_name[2] = '0';								//垂直线绘制
	memcpy(data_pack, (uint8_t*)line_drawing(_layer,ADD_PICTURE,center_x, scale_point[0], center_x, scale_point[6], 3, ruler_color, line_name), DRAWING_PACK);
	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);

	for(i = 0;i < 7;i++)
	{
		line_name[2] += (i + 1);
		if(i % 2 == 0)								//奇数刻度线，短线
			memcpy(&data_pack[DRAWING_PACK*i], (uint8_t*)line_drawing(_layer,ADD_PICTURE,center_x-short_scale_length/2, scale_point[i], center_x+short_scale_length/2, scale_point[i], 3, ruler_color, line_name), DRAWING_PACK);
		else										//偶数刻度线，长线
			memcpy(&data_pack[DRAWING_PACK*i], (uint8_t*)line_drawing(_layer,ADD_PICTURE,center_x-long_scale_length/2, scale_point[i], center_x+long_scale_length/2, scale_point[i], 3, ruler_color, line_name), DRAWING_PACK);
	}
	pack_send_robotData(Drawing_7_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*7);

	/* 生成每个小刻度线 */
	for(i = 1;i < 7;i++)
	{
		line_name[1] += i;
		line_name[2] = '0';

		for(uint8_t j = 0;j < 4;j++)
		{
			line_name[2] += j;
			memcpy(&data_pack[DRAWING_PACK*j], (uint8_t*)line_drawing(_layer,ADD_PICTURE,center_x-short_scale_length/4, scale_point[i]+scale_step*(j+1), center_x+short_scale_length/4, scale_point[i]+scale_step*(j+1), 3, ruler_color, line_name), DRAWING_PACK);
		}
		//空包
		line_name[2] = '4';
		memcpy(&data_pack[DRAWING_PACK*4], (uint8_t*)null_drawing(0, line_name), DRAWING_PACK);
		pack_send_robotData(Drawing_5_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*5);	
	}

	/* 绘制标尺大刻度对应数字 */
	line_name[1] = 'c';	
	line_name[2] = '0';
	for(i = 0;i < 7;i++)
	{
		line_name[2] += i;
		memcpy(&data_pack[DRAWING_PACK*i], (uint8_t*)int_drawing(_layer, ADD_PICTURE, center_x+long_scale_length, scale_point[i] + 8, 16, 2, ruler_color, (15 - 5*i)*1000, line_name), DRAWING_PACK);
	}
	pack_send_robotData(Drawing_7_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*7);
	
	/* 绘制空中机器人浮标目标值 */
	line_name[1] = 'p';
	line_name[2] = '0';
	memcpy(data_pack, (uint8_t*)circle_drawing(_layer, ADD_PICTURE, center_x,center_y , 10, 16, target_color, line_name), DRAWING_PACK);
	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);	
	
	/* 绘制空中机器人初始浮标 */
	line_name[2] = '1';
	memcpy(data_pack, (uint8_t*)circle_drawing(_layer, ADD_PICTURE, center_x,center_y , 10, 16, current_color, line_name), DRAWING_PACK);
	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);	
}

/**
 * @brief 【自定义图层】空中机器人pitch浮标当前值
 * @note 画图有问题的话可能是name重复了
 * @note 
 */
void referee_Classdef::Pitch_Current_Point(float pitch_angle, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, colorType_e tag_color)
{
	static uint8_t point_str[] = "6p1";
	
	pitch_angle = _referee_Constrain((float)pitch_angle, (float)(-15.0), (float)(15.0));
	
	/* 计算浮标点坐标位置并绘制 */
	memcpy(data_pack, (uint8_t*)circle_drawing(_layer, MODIFY_PICTURE, center_x, center_y + pitch_angle*total_length/30, 10, 16, tag_color, point_str), DRAWING_PACK);
	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);	
	
}

/**
 * @brief 【自定义图层】空中机器人pitch浮标目标值
 * @note 画图有问题的话可能是name重复了
 * @note 
 */
void referee_Classdef::Pitch_Target_Point(float pitch_target, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, colorType_e tag_color)
{
	static uint8_t point_str[] = "6p0";
	pitch_target = _referee_Constrain((float)pitch_target, (float)(-15.0), (float)(15.0));
	
	/* 计算浮标点坐标位置并绘制 */
	memcpy(data_pack, (uint8_t*)circle_drawing(_layer, MODIFY_PICTURE, center_x, center_y + pitch_target*total_length/30, 10, 16, tag_color, point_str), DRAWING_PACK);
	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);	
}

/**
 * @brief 【自定义图层】工程机器人抬升高度标尺
 * @note 画图有问题的话可能是name重复了
 * @note 
 */
void referee_Classdef::Engineer_Height_Ruler(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, uint16_t long_scale_length, uint16_t short_scale_length, uint8_t ruler_tag, colorType_e ruler_color, colorType_e current_color)
{
	uint8_t line_name[] = "200";					//Engineer id + tag

	uint16_t scale_step = total_length / 20;
	uint16_t scale_point[21];						//存放水平刻度线的垂直坐标
	uint8_t i = 0;									//计数变量

	/* 生成每个刻度线的垂直坐标 */
	scale_point[0] = center_y + total_length/2;		//第一条刻度
	
	for(i = 1;i < 21;i++)							//其他刻度
		scale_point[i] = scale_point[0] - total_length/20*i;
	
	line_name[1] = '0' + ruler_tag;
	line_name[2] = 'a';								//垂直线绘制
	memcpy(data_pack, (uint8_t*)line_drawing(_layer,ADD_PICTURE,center_x, scale_point[0], center_x, scale_point[20], 3, ruler_color, line_name), DRAWING_PACK);
	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);

	/* 水平刻度线绘制 */

	line_name[2] += 1;
	for(i = 0;i < 21;i++)
	{
		line_name[2] += i;
		if(i % 2 == 0)								//偶数条水平刻度线，长线
			memcpy(&data_pack[DRAWING_PACK*(i%7)], (uint8_t*)line_drawing(_layer,ADD_PICTURE,center_x-long_scale_length/2, scale_point[i], center_x+long_scale_length/2, scale_point[i], 3, ruler_color, line_name), DRAWING_PACK);
		else										//奇数条水平刻度线，短线
			memcpy(&data_pack[DRAWING_PACK*(i%7)], (uint8_t*)line_drawing(_layer,ADD_PICTURE,center_x-short_scale_length/2, scale_point[i], center_x+short_scale_length/2, scale_point[i], 3, ruler_color, line_name), DRAWING_PACK);
	
		if((i+1)%7 == 0)							//装满七个图形则绘制一次
			pack_send_robotData(Drawing_7_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*7);
	}

	/* 绘制标尺大刻度对应数字 */
	line_name[1] = 'c' + ruler_tag;	
	line_name[2] = 'a';														//这里从小写字母a开始遍历
	for(i = 0;i < 7;i++)
	{
		line_name[2] += i;
		memcpy(&data_pack[DRAWING_PACK*i], (uint8_t*)int_drawing(_layer, ADD_PICTURE, center_x+long_scale_length, scale_point[i*2] + 8, 16, 2, ruler_color, (10-i)*10000, line_name), DRAWING_PACK);
	}
	pack_send_robotData(Drawing_7_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*7);
	
	for(i = 7;i < 11;i++)
	{
		line_name[2] += (i-6);
		memcpy(&data_pack[DRAWING_PACK*(i-7)], (uint8_t*)int_drawing(_layer, ADD_PICTURE, center_x+long_scale_length, scale_point[i*2] + 8, 16, 2, ruler_color, (10-i)*10000, line_name), DRAWING_PACK);
	}
	//空包
	line_name[2] += 1;
	memcpy(&data_pack[DRAWING_PACK*4], (uint8_t*)null_drawing(0, line_name), DRAWING_PACK);
	pack_send_robotData(Drawing_5_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*5);	

	/* 绘制高度初始浮标 */
	line_name[1] = 'h' + ruler_tag;
	line_name[2] = '1';
	memcpy(data_pack, (uint8_t*)circle_drawing(_layer, ADD_PICTURE, center_x,center_y , 10, 18, current_color, line_name), DRAWING_PACK);
	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);	
}

/**
 * @brief 【自定义图层】工程机器人当前抬升高度浮标
 * @note 画图有问题的话可能是name重复了
 * @note 
 */
void referee_Classdef::Engineer_Current_Height(float height, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, uint8_t ruler_tag, colorType_e tag_color)
{
	static uint8_t point_str[] = "2h1";
	
	height = _referee_Constrain((float)height, (float)(0.0), (float)(1.0));
	
	/* 计算浮标点坐标位置并绘制 */
	point_str[1] = 'h' + ruler_tag;				//第几条标尺
	memcpy(data_pack, (uint8_t*)circle_drawing(_layer, MODIFY_PICTURE, center_x, center_y + (height - 0.5)*(float)total_length, 10, 16, tag_color, point_str), DRAWING_PACK);
	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);		
}

/**
 * @brief 【自定义图层】工程机器人当前抬升高度浮标（两个浮标同时绘制，缩短时间）
 * @note 画图有问题的话可能是name重复了
 * @note 
 */
void referee_Classdef::Engineer_Current_HeightMul(float *height, uint8_t _layer, uint16_t *center_x, uint16_t *center_y, uint16_t *total_length, colorType_e tag_color)
{
	static uint8_t point_str[] = "2h1";
	
	point_str[1] = 'h';
	point_str[2] = '1';
	
	for(uint8_t i = 0;i < 2;i++)
	{
		height[i] = _referee_Constrain((float)height[i], (float)(0.0), (float)(1.0));
		/* 计算浮标点坐标位置并绘制 */
		point_str[1] = 'h' + i;				//第i条标尺
		memcpy(&data_pack[DRAWING_PACK*i], (uint8_t*)circle_drawing(_layer, MODIFY_PICTURE, center_x[i], center_y[i] + (height[i] - 0.5)*(float)total_length[i], 10, 16, tag_color, point_str), DRAWING_PACK);
		
	}

	pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);	
}

/* 工程机器人目标抬升高度浮标 */
void referee_Classdef::Engineer_Target_Height(float *target_height, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, uint8_t ruler_tag, colorType_e tag_color)
{

}

/**
 * @brief 【自定义图层】矿石UI
 * @note 画图有问题的话可能是name重复了
 * @note 
 */
void referee_Classdef::Mine(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t mine_tag, colorType_e _color)
{	
	uint8_t mine_name[] = "2ma";
	uint8_t mine_r = 'R';

	/* 方框 */
	mine_name[2] += mine_tag;
	memcpy(data_pack, (uint8_t*)rectangle_drawing(_layer, ADD_PICTURE, center_x - size/2, center_y - size/2, size, size, size/10, _color, mine_name), DRAWING_PACK);
	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);

	/* 字符R */
	mine_name[2] -= 0x20;
	Draw_Char(_layer, center_x - size/10, center_y + size/5, mine_name, &mine_r, 1, size/2, _color, ADD_PICTURE);
}

/**
 * @brief 【自定义图层】工程机器人当前存储矿石数UI框架
 * @note 画图有问题的话可能是name重复了
 * @note 
 */
void referee_Classdef::Mine_Remain_Frame(uint8_t _layer, uint16_t start_x, uint16_t start_y)
{
	uint8_t mine_str[] = ":0";
	uint8_t char_str[] = "2m0";
	
	
	Mine(_layer, start_x + 30, start_y - 30, 60, 1, YELLOW);
	Draw_Char(_layer, start_x + 85, start_y - 20, char_str, mine_str, 2, 40, YELLOW, ADD_PICTURE);	
}

/**
 * @brief 【自定义图层】工程机器人当前存储矿石数
 * @note 画图有问题的话可能是name重复了
 * @note 
 */
void referee_Classdef::Mine_Remain_Update(uint8_t mine_num, uint8_t _layer, uint16_t start_x, uint16_t start_y)
{
	static uint8_t mine_str[] = ":0";
	static uint8_t char_str[] = "2m0";
	static uint8_t last_mine_num = 255;
	
	if(mine_num == last_mine_num)
		return;

	/* 矿物数目更新 */
	mine_str[1] = '0' + mine_num;
	Draw_Char(_layer, start_x + 85, start_y - 20, char_str, mine_str, 2, 40, YELLOW, MODIFY_PICTURE);
	
	last_mine_num = mine_num;

}

/**
 * @brief 【自定义图层】工程机器人自动/手动组状态 
 * @note 画图有问题的话可能是name重复了
 * @note 
 */
void referee_Classdef::Engineer_Miner_Mode(uint8_t mode, uint8_t _layer, uint16_t start_x, uint16_t start_y, colorType_e _color)
{
	static uint8_t auto_str[] = "AUTO MODE";
	static uint8_t handle_str[] = "HANDLE_MODE";
	static uint8_t mode_str[] = "2M0";
	static uint8_t last_mode = 255;
	
	if(mode == last_mode)
		return;
	
	if(mode)				//自动模式
	{
		clean_layer(_layer);
		Draw_Char(_layer, start_x, start_y, mode_str, auto_str, sizeof(auto_str), 20, _color, ADD_PICTURE);
	}
	else
	{
		clean_layer(_layer);
		Draw_Char(_layer, start_x, start_y, mode_str, handle_str, sizeof(handle_str), 20, _color, ADD_PICTURE);
	}
	
	last_mode = mode;
}

/**
 * @brief 【自定义图层】工程机器人当前取矿状态
 * @note 画图有问题的话可能是name重复了
 * @note 
 */
void referee_Classdef::Engineer_Miner_Status(uint8_t status, uint8_t _layer, uint16_t start_x, uint16_t start_y)
{
	static uint8_t Exchange_str[] = "2e0";
	static uint8_t Rotate_str[] = "2r0";
	static uint8_t Reach_str[] = "2R0";
	static uint8_t last_status = 255;
	
	static uint8_t i = 3;
	
	Rotate_str[2] = '0';
	Reach_str[2] = '0';
	
	
	if(status == last_status)
	{
		i = 6;
		return;
	}
	else
	{
		while(i--)
		{
			switch(status)
			{
				case 0: 				//取金矿
					clean_layer(_layer);
					Mine(_layer, start_x + 30, start_y - 30, 60, 2, YELLOW);
					break;
				case 1: 				//取银矿
					clean_layer(_layer);
					Mine(_layer, start_x + 30, start_y - 30, 60, 2, WHITE);
					break;
				case 2: 				//兑换矿物
					clean_layer(_layer);
					Mine(_layer, start_x + 30, start_y - 30, 60, 2, GREEN);
					memcpy(data_pack, (uint8_t*)line_drawing(_layer,ADD_PICTURE,start_x - 20, start_y - 30, start_x + 30, start_y - 30, 20, GREEN, Exchange_str), DRAWING_PACK);
					pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);
					break;
				case 3: 				//伸出
					clean_layer(_layer);
					memcpy(data_pack, (uint8_t*)line_drawing(_layer,ADD_PICTURE,start_x, start_y - 30, start_x + 80, start_y - 30, 6, WHITE, Reach_str), DRAWING_PACK);
					Reach_str[2] += 1;	//上箭头
					memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(_layer,ADD_PICTURE,start_x + 60, start_y - 10, start_x + 80, start_y - 30, 6, WHITE, Reach_str), DRAWING_PACK);
					Reach_str[2] += 1;	//下箭头
					memcpy(&data_pack[DRAWING_PACK*2], (uint8_t*)line_drawing(_layer,ADD_PICTURE,start_x + 60, start_y - 50, start_x + 80, start_y - 30, 6, WHITE, Reach_str), DRAWING_PACK);
					
					Reach_str[2] += 1;	//空包
					memcpy(&data_pack[DRAWING_PACK*3], (uint8_t*)null_drawing(_layer, Reach_str),DRAWING_PACK);
					Reach_str[2] += 1;
					memcpy(&data_pack[DRAWING_PACK*4], (uint8_t*)null_drawing(_layer, Reach_str),DRAWING_PACK);		
				
					pack_send_robotData(Drawing_5_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*5);
					break;
				
				case 4: 				//翻转
					clean_layer(_layer);
					memcpy(data_pack, (uint8_t*)line_drawing(_layer,ADD_PICTURE,start_x, start_y - 30, start_x + 60, start_y - 30, 6, WHITE, Rotate_str), DRAWING_PACK);
					Rotate_str[2] += 1;	//上箭头
					memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(_layer,ADD_PICTURE,start_x + 60, start_y - 30, start_x + 100, start_y + 10, 6, WHITE, Rotate_str), DRAWING_PACK);
					Rotate_str[2] += 1;	//下箭头
					memcpy(&data_pack[DRAWING_PACK*2], (uint8_t*)circle_drawing(_layer,ADD_PICTURE, start_x + 60, start_y - 30, 16, 6, WHITE, Rotate_str), DRAWING_PACK);
					
					Rotate_str[2] += 1;	//空包
					memcpy(&data_pack[DRAWING_PACK*3], (uint8_t*)null_drawing(_layer, Rotate_str),DRAWING_PACK);
					Rotate_str[2] += 1;
					memcpy(&data_pack[DRAWING_PACK*4], (uint8_t*)null_drawing(_layer, Rotate_str),DRAWING_PACK);		
				
					pack_send_robotData(Drawing_5_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*5);
					break;
				default:
					break;
			}
		}
	}

	last_status = status;
}

/**
 * @brief 【自定义图层】哨兵机器人移动轨道绘制
 * @note 画图有问题的话可能是name重复了
 * @note 
 */
void referee_Classdef::Sentry_Position_Ruler(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, colorType_e ruler_color, colorType_e current_color)
{
	uint8_t line_name[] = "7r0";

	//哨兵轨道
	memcpy(data_pack, (uint8_t*)line_drawing(_layer,ADD_PICTURE,center_x - total_length/2, center_y, center_x + total_length/2, center_y, 3, ruler_color, line_name), DRAWING_PACK);
	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);

	//哨兵初始底盘位置
	line_name[2] += 1;
	memcpy(data_pack, (uint8_t*)circle_drawing(_layer, ADD_PICTURE, center_x,center_y , 10, 18, current_color, line_name), DRAWING_PACK);
	line_name[2] += 1;
	memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(_layer,ADD_PICTURE,center_x, center_y + 40, center_x, center_y - 40, 6, current_color, line_name), DRAWING_PACK);
	pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);
}

/**
 * @brief 【自定义图层】哨兵机器人当前位置浮标
 * @note 画图有问题的话可能是name重复了
 * @note 
 */
void referee_Classdef::Sentry_Current_Pos(float pos_percent, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, colorType_e tag_color)
{
	static uint8_t pos_name[] = "7r1";
	static uint16_t current_pos = 0;

	pos_percent = _referee_Constrain((float)pos_percent, (float)0.0, (float)1.0);
	current_pos = (pos_percent - 0.5)*(float)total_length + center_x;

	pos_name[2] = '1';
	memcpy(data_pack, (uint8_t*)circle_drawing(_layer, MODIFY_PICTURE, current_pos, center_y , 10, 18, tag_color, pos_name), DRAWING_PACK);
	pos_name[2] += 1;
	memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(_layer, MODIFY_PICTURE, current_pos, center_y + 40, current_pos, center_y - 40, 6, tag_color, pos_name), DRAWING_PACK);
	pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);
}

/**
 * @brief 【自定义图层】哨兵机器人巡逻区域绘制
 * @note 画图有问题的话可能是name重复了
 * @note 
 */
void referee_Classdef::Sentry_Patrol_Area(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t size, uint8_t *keys, colorType_e patrol_color)
{
	uint8_t line_name[] = "7a0";

	/* 绘制区域底色 */
	memcpy(data_pack, (uint8_t*)line_drawing(_layer, ADD_PICTURE, center_x - size/2, center_y, center_x + size/2, center_y, size, WHITE, line_name), DRAWING_PACK);
	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);

	/* 绘制区域方框 */
	//中心线
	line_name[2] += 1;
	memcpy(data_pack, (uint8_t*)line_drawing(_layer, ADD_PICTURE, center_x - size/2, center_y, center_x + size/2, center_y, 3, BLACK, line_name), DRAWING_PACK);
	line_name[2] += 1;
	memcpy(&data_pack[DRAWING_PACK], (uint8_t*)line_drawing(_layer, ADD_PICTURE, center_x, center_y - size/2, center_x, center_y + size/2, 3, BLACK, line_name), DRAWING_PACK);
	pack_send_robotData(Drawing_2_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK*2);

	//方框
	line_name[2] += 1;
	memcpy(data_pack, (uint8_t*)rectangle_drawing(_layer, ADD_PICTURE, center_x - size/2, center_y - size/2, size, size, 3, BLACK, line_name), DRAWING_PACK);
	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);

	//字符
	line_name[2] += 1;
	Draw_Char(_layer, center_x-size*0.4, center_y+size*0.4, line_name, keys, 1, size*0.3, BLACK, ADD_PICTURE);
	line_name[2] += 1;
	Draw_Char(_layer, center_x-size*0.4, center_y-size*0.1, line_name, keys+1, 1, size*0.3, BLACK, ADD_PICTURE);
	line_name[2] += 1;
	Draw_Char(_layer, center_x+size*0.1, center_y+size*0.4, line_name, keys+2, 1, size*0.3, BLACK, ADD_PICTURE);
	line_name[2] += 1;
	Draw_Char(_layer, center_x+size*0.1, center_y-size*0.1, line_name, keys+3, 1, size*0.3, BLACK, ADD_PICTURE);

	/* 绘制初始哨兵巡逻区域，绘制在左上角 */
	line_name[2] += 1;
	memcpy(data_pack, (uint8_t*)line_drawing(_layer, ADD_PICTURE, center_x - size/2, center_y + size/4, center_x, center_y + size/4, size/2, patrol_color, line_name), DRAWING_PACK);
	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);

}

/**
 * @brief 【自定义图层】哨兵机器人索敌状态指示
 * @note 画图有问题的话可能是name重复了
 * @note 
 */
void referee_Classdef::Sentry_Patrol_Status(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t size, uint8_t tag, colorType_e _color)
{
	static uint8_t patrol_area[] = "7a8";
	static uint8_t last_patrol_tag = 255;
	static uint8_t i = 3;
	
	if(last_patrol_tag == tag)
	{
		i = 3;
		return;
	}
	
	while(i--)
	{
		switch (tag)
		{
		case 1:				//左上
			memcpy(data_pack, (uint8_t*)line_drawing(_layer, MODIFY_PICTURE, center_x - size/2, center_y + size/4, center_x, center_y + size/4, size/2, _color, patrol_area), DRAWING_PACK);
			break;
		case 2:				//右上
			memcpy(data_pack, (uint8_t*)line_drawing(_layer, MODIFY_PICTURE, center_x, center_y + size/4, center_x + size/2, center_y + size/4, size/2, _color, patrol_area), DRAWING_PACK);
			break;
		case 3:				//左下
			memcpy(data_pack, (uint8_t*)line_drawing(_layer, MODIFY_PICTURE, center_x - size/2, center_y - size/4, center_x, center_y - size/4, size/2, _color, patrol_area), DRAWING_PACK);
			break;
		case 4:				//右下
			memcpy(data_pack, (uint8_t*)line_drawing(_layer, MODIFY_PICTURE, center_x, center_y - size/4, center_x + size/2, center_y - size/4, size/2, _color, patrol_area), DRAWING_PACK);
			break;	
		default:
			break;
		}
		pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)data_pack, DRAWING_PACK);
	}
	
	last_patrol_tag = tag;
}

/**
 * @brief 【自定义图层】哨兵机器人弹丸状态框架
 * @note 画图有问题的话可能是name重复了
 * @note 
 */
void referee_Classdef::Sentry_Bullet_Frame(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t size, colorType_e _color)
{
	uint8_t bullet_name[] = "7b0";
	uint8_t bullet_str[] = "Freq:      ";
	
	Draw_Char(_layer, start_x, start_y, bullet_name, bullet_str, sizeof(bullet_str), size, _color, ADD_PICTURE);
}

/**
 * @brief 【自定义图层】哨兵机器人弹丸实时状态
 * @note 画图有问题的话可能是name重复了
 * @note 
 */
void referee_Classdef::Sentry_Bullet_Status(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t size, uint8_t tag, colorType_e _color)
{
	static uint8_t bullet_name[] = "7b0";
	static uint8_t bullet_str[] = "Freq:      ";
	
	static uint8_t last_tag = 255;
	static uint8_t i = 3;
	
	if(last_tag == tag)
	{
		i = 3;
		return;
	}
	else
	{
		switch(tag)
		{
			case 0:
				memcpy(&bullet_str[5], "Normal", 6);
				break;
			case 1:
				memcpy(&bullet_str[5], "Low   ", 6);
				break;
			case 2:
				memcpy(&bullet_str[5], "No!!  ", 6);
				break;			
		}
		
		while(i--)
		{
			Draw_Char(_layer, start_x, start_y, bullet_name, bullet_str, sizeof(bullet_str), size, _color, MODIFY_PICTURE);
		}
	}
	
	last_tag = tag;
	
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
