/***********************************************
Lightweight lidar mapping navigation mode.
***********************************************/

#ifndef __LIDAR_MAPPING_NAV_H
#define __LIDAR_MAPPING_NAV_H

#include "main.h"

#define MAP_NAV_STATE_CRUISE       0
#define MAP_NAV_STATE_AVOID        1
#define MAP_NAV_STATE_BACK         2
#define MAP_NAV_STATE_RECOVER_TURN 3
#define MAP_NAV_STATE_PAUSE        4

typedef struct
{
	float x_mm;
	float y_mm;
	float theta_rad;
}MapNavPose;

typedef struct
{
	u8 state;
	u16 nearest_mm;
	float nearest_angle_deg;
	float best_angle_deg;
	u16 best_clear_mm;
	u16 map_updates;
	u16 no_data_ticks;
	u8 occupied_cells;
}MapNavTelemetry;

extern MapNavPose MapNav_Pose;
extern MapNavTelemetry MapNav_Telemetry;

void Lidar_Mapping_Nav_Reset(void);
void Lidar_Mapping_Nav_Task(void);

#endif
