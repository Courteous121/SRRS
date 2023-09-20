#ifndef __DATASTRUCT_
#define __DATASTRUCT_


#include <stdint.h> 
#include <bits/types.h>
#include <memory>


/**
 * @brief 单云台协议结构体
 */
#pragma pack(1)
typedef struct
{ 
    uint8_t index;
    uint8_t R1x;
    uint8_t R1y;
    uint8_t R2x;
    uint8_t R2y;
    uint8_t R3x;
    uint8_t R3y;
    uint8_t R4x;
    uint8_t R4y;
    uint8_t R5x;
    uint8_t R5y;
    uint8_t B1x;
    uint8_t B1y;
    uint8_t B2x;
    uint8_t B2y;
    uint8_t B3x;
    uint8_t B3y;
    uint8_t B4x;
    uint8_t B4y;
    uint8_t B5x;
    uint8_t B5y;
}PackToGroundRobotDef;

#pragma pack()
typedef union 
{
	uint8_t UsartData[21];
	PackToGroundRobotDef PackToGroundRobot;
}PackToGroundRobotUnionDef;

extern PackToGroundRobotUnionDef PackToGroundRobotUnion;

#pragma pack(1)
typedef struct
{
    uint8_t carnum;
    uint8_t isMissile;
    uint8_t CarDensityMax;
    uint8_t EnermyToDead;
}PackToSentryDef;

#pragma pack()
typedef union 
{
	uint8_t UsartData[5];
	PackToSentryDef PackToSentry;
}PackToSentryUnionDef;

extern PackToSentryUnionDef PackToSentryUnion;
//共用包
#pragma pack(1)
typedef struct
{ 
    uint8_t protect;
    uint8_t attack;
    uint8_t event;
    uint8_t suggestion;
}PackToGroundRobotStrategyDef;

#pragma pack()
typedef union 
{
	uint8_t UsartData[4];
	PackToGroundRobotStrategyDef PackToGroundRobotStrat;
}PackToGroundRobotStratUnionDef;

extern PackToGroundRobotStratUnionDef PackToGroundRobotStratUnion;


//专属包
#pragma pack(1)
typedef struct
{ 
    uint8_t suggestion;
}PackToHeroDef;

#pragma pack()
typedef union 
{
	uint8_t UsartData[1];
	PackToHeroDef PackToHero;
}PackToHeroUnionDef;

extern PackToHeroUnionDef PackToHeroUnion;
#pragma pack(1)
typedef struct
{ 

    uint8_t suggestion;
}PackToInfantryDef;

#pragma pack()
typedef union 
{
	uint8_t UsartData[1];
	PackToInfantryDef PackToInfantry;
}PackToInfantryUnionDef;

extern PackToInfantryUnionDef PackToInfantryUnion;
#pragma pack(1)
typedef struct
{ 

    uint8_t suggestion;
}PackToEngineerDef;

#pragma pack()
typedef union 
{
	uint8_t UsartData[1];
	PackToEngineerDef PackToEngineer;
}PackToEngineerUnionDef;

extern PackToEngineerUnionDef PackToEngineerUnion;

#endif // !__DATASTRUCT_
