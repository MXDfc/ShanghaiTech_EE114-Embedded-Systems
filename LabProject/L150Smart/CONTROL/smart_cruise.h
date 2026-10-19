/***********************************************
Smart cruise obstacle avoidance for EE114 project.
***********************************************/

#ifndef __SMART_CRUISE_H
#define __SMART_CRUISE_H

#include "main.h"

#define SMART_STATE_CRUISE_FORWARD 0
#define SMART_STATE_AVOID_SLOW     1
#define SMART_STATE_AVOID_TURN     2
#define SMART_STATE_ESCAPE_BACK    3
#define SMART_STATE_PAUSE_STOP     4
#define SMART_STATE_EXIT_STRAIGHT  5

typedef struct
{
	u16 front_mm;
	u16 front_left_mm;
	u16 front_right_mm;
	u16 left_mm;
	u16 right_mm;
	u16 left_rear_mm;
	u16 right_rear_mm;
	u16 rear_mm;
	u8 valid;
}SmartCruiseSectors;

typedef struct
{
	float cruise_speed;
	float slow_speed;
	float back_speed;
	float max_turn_angle;
	u16 warn_distance_mm;
	u16 stop_distance_mm;
	u16 escape_distance_mm;
	u16 no_data_timeout_ticks;
}SmartCruiseConfig;

typedef struct
{
	u8 state;
	u16 nearest_mm;
	float nearest_angle;
	u16 obstacle_count;
	u16 no_data_ticks;
	u8 turn_dir;
}SmartCruiseTelemetry;

typedef struct
{
	float front_offset_deg;
	float front_min_deg;
	float front_max_deg;
	float front_hit_angle_deg;
	float nearest_raw_angle_deg;
	float nearest_body_angle_deg;
	u16 front_hit_mm;
	u16 front_point_count;
}SmartCruiseDebug;

extern SmartCruiseSectors SmartCruise_Sectors;
extern SmartCruiseConfig SmartCruise_Config;
extern SmartCruiseTelemetry SmartCruise_Telemetry;
extern SmartCruiseDebug SmartCruise_Debug;
extern float Smart_Front_Angle_Offset;
extern volatile float Watch_Front_Offset_Deg;
extern volatile float Watch_Front_Hit_Angle_Deg;
extern volatile float Watch_Nearest_Raw_Angle_Deg;
extern volatile float Watch_Nearest_Body_Angle_Deg;
extern volatile int Watch_Front_Offset_X10;
extern volatile int Watch_Front_Hit_Angle_X10;
extern volatile int Watch_Nearest_Raw_Angle_X10;
extern volatile int Watch_Nearest_Body_Angle_X10;
extern volatile u16 Watch_Front_Hit_MM;
extern volatile u16 Watch_Front_Point_Count;
extern volatile int Watch_Front_Window_Min_X10;
extern volatile int Watch_Front_Window_Max_X10;

void Smart_Cruise_Reset(void);
void Smart_Cruise_Task(void);
void Smart_Cruise_UpdateSectors(void);

#endif
