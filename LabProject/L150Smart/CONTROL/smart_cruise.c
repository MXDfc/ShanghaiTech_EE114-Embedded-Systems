/***********************************************
Smart cruise obstacle avoidance for EE114 project.
***********************************************/

#include "smart_cruise.h"
#include "Header.h"
#include "Lidar.h"

#if LD14P
extern LidarPointStructDef Dataprocess[800];
#else
extern PointDataProcessDef Dataprocess[400];
#endif

SmartCruiseSectors SmartCruise_Sectors;
float Smart_Front_Angle_Offset = 0.0f;
SmartCruiseConfig SmartCruise_Config = {
	0.25f,   /* cruise_speed */
	0.10f,   /* slow_speed */
	-0.14f,  /* back_speed */
	0.38f,   /* max_turn_angle, rad for Ackermann front wheel */
	450,     /* warn_distance_mm */
	300,     /* stop_distance_mm */
	300,     /* escape_distance_mm */
	80       /* no_data_timeout_ticks, 400 ms at 5 ms/tick */
};
SmartCruiseTelemetry SmartCruise_Telemetry;
SmartCruiseDebug SmartCruise_Debug;
volatile float Watch_Front_Offset_Deg;
volatile float Watch_Front_Hit_Angle_Deg;
volatile float Watch_Nearest_Raw_Angle_Deg;
volatile float Watch_Nearest_Body_Angle_Deg;
volatile int Watch_Front_Offset_X10;
volatile int Watch_Front_Hit_Angle_X10;
volatile int Watch_Nearest_Raw_Angle_X10;
volatile int Watch_Nearest_Body_Angle_X10;
volatile u16 Watch_Front_Hit_MM;
volatile u16 Watch_Front_Point_Count;
volatile int Watch_Front_Window_Min_X10;
volatile int Watch_Front_Window_Max_X10;

#define SMART_TURN_LEFT                 1
#define SMART_TURN_RIGHT               -1
#define SMART_LIDAR_MIN_VALID_MM       30
#define SMART_LIDAR_NEAR_CLAMP_MM      80
#define SMART_LIDAR_MAX_VALID_MM     5000
#define SMART_FRONT_MIN_DEG           -20.0f
#define SMART_FRONT_MAX_DEG            20.0f
#define SMART_RIGHT_MIN_DEG           -90.0f
#define SMART_RIGHT_MAX_DEG           -20.0f
#define SMART_LEFT_MIN_DEG             20.0f
#define SMART_LEFT_MAX_DEG             90.0f
#define SMART_CLEAR_MARGIN_MM          80
#define SMART_ESCAPE_BACK_TICKS        90
#define SMART_ESCAPE_TURN_TICKS       120
#define SMART_AVOID_TIMEOUT_TICKS      260
#define SMART_MIN_TURN_TICKS           70
#define SMART_CLEAR_CONFIRM_TICKS      18

static u16 state_tick = 0;
static u16 clear_tick = 0;
static int turn_direction = 0;
static u8 surround_escape_active = 0;

static float Smart_Normalize_Angle(float angle)
{
	while(angle > 180.0f) angle -= 360.0f;
	while(angle < -180.0f) angle += 360.0f;
	return angle;
}

static u8 Smart_Angle_In_Range(float angle,float min_angle,float max_angle)
{
	angle = Smart_Normalize_Angle(angle);
	return (angle >= min_angle && angle <= max_angle);
}

static int Smart_Angle_To_X10(float angle)
{
	return (int)(angle * 10.0f);
}

static void Smart_Update_Min(u16 *min_distance,u16 distance)
{
	if(distance > 0 && distance < *min_distance)
		*min_distance = distance;
}

static u16 Smart_Finalize_Min(u16 distance)
{
	return (distance == 65535) ? 0 : distance;
}

static u16 Smart_Open_Space(u16 distance)
{
	return (distance == 0) ? SMART_LIDAR_MAX_VALID_MM : distance;
}

static u8 Smart_Distance_Close(u16 distance,u16 threshold)
{
	return (distance > 0 && distance < threshold);
}

static u8 Smart_Front_Close(void)
{
	return Smart_Distance_Close(SmartCruise_Sectors.front_mm,
		SmartCruise_Config.stop_distance_mm);
}

static u8 Smart_Front_Clear(void)
{
	return (SmartCruise_Sectors.front_mm == 0 ||
		SmartCruise_Sectors.front_mm >
		(u16)(SmartCruise_Config.stop_distance_mm + SMART_CLEAR_MARGIN_MM));
}

static int Smart_Select_Turn_Direction(void)
{
	u16 left_space = Smart_Open_Space(SmartCruise_Sectors.front_left_mm);
	u16 right_space = Smart_Open_Space(SmartCruise_Sectors.front_right_mm);

	if(turn_direction != 0)
		return turn_direction;

	return (left_space >= right_space) ? SMART_TURN_LEFT : SMART_TURN_RIGHT;
}

static u8 Smart_Left_Blocked(void)
{
	return Smart_Distance_Close(SmartCruise_Sectors.front_left_mm,
		SmartCruise_Config.stop_distance_mm);
}

static u8 Smart_Right_Blocked(void)
{
	return Smart_Distance_Close(SmartCruise_Sectors.front_right_mm,
		SmartCruise_Config.stop_distance_mm);
}

static u8 Smart_Surrounded(void)
{
	return Smart_Front_Close() && Smart_Left_Blocked() && Smart_Right_Blocked();
}

static void Smart_Set_Turn_Command(void)
{
	turn_direction = Smart_Select_Turn_Direction();
	SmartCruise_Telemetry.turn_dir = (turn_direction == SMART_TURN_LEFT) ? 1 : 2;
	Move_X = SmartCruise_Config.slow_speed;
	Move_Z = (turn_direction == SMART_TURN_LEFT) ?
		SmartCruise_Config.max_turn_angle : -SmartCruise_Config.max_turn_angle;
}

void Smart_Cruise_Reset(void)
{
	SmartCruise_Telemetry.state = SMART_STATE_CRUISE_FORWARD;
	SmartCruise_Telemetry.nearest_mm = 0;
	SmartCruise_Telemetry.nearest_angle = 0.0f;
	SmartCruise_Telemetry.obstacle_count = 0;
	SmartCruise_Telemetry.no_data_ticks = 0;
	SmartCruise_Telemetry.turn_dir = 0;
	SmartCruise_Sectors.valid = 0;
	state_tick = 0;
	clear_tick = 0;
	turn_direction = 0;
	surround_escape_active = 0;
	Move_X = 0;
	Move_Z = 0;
}

void Smart_Cruise_UpdateSectors(void)
{
	int i,count;
	u16 distance;
	float angle;
	float raw_angle;
	u16 front = 65535;
	u16 front_left = 65535;
	u16 front_right = 65535;
	u16 nearest = 65535;
	float nearest_angle = 0.0f;
	float nearest_raw_angle = 0.0f;
	u16 front_count = 0;
	u16 front_hit = 65535;
	float front_hit_angle = 0.0f;
	u8 valid = 0;

	count = lap_count;
#if LD14P
	if(count > 800) count = 800;
#else
	if(count > 400) count = 400;
#endif

	for(i=0;i<count;i++)
	{
		distance = Dataprocess[i].distance;
		if(distance < SMART_LIDAR_MIN_VALID_MM ||
			distance > SMART_LIDAR_MAX_VALID_MM)
			continue;
		if(distance < SMART_LIDAR_NEAR_CLAMP_MM)
			distance = SMART_LIDAR_NEAR_CLAMP_MM;

		valid = 1;
		raw_angle = Smart_Normalize_Angle(Dataprocess[i].angle);
		angle = Smart_Normalize_Angle(raw_angle - Smart_Front_Angle_Offset);

		if(distance < nearest)
		{
			nearest = distance;
			nearest_angle = angle;
			nearest_raw_angle = raw_angle;
		}

		if(Smart_Angle_In_Range(angle,SMART_FRONT_MIN_DEG,SMART_FRONT_MAX_DEG))
		{
			Smart_Update_Min(&front,distance);
			front_count++;
			if(distance < front_hit)
			{
				front_hit = distance;
				front_hit_angle = angle;
			}
		}
		else if(Smart_Angle_In_Range(angle,SMART_LEFT_MIN_DEG,SMART_LEFT_MAX_DEG))
			Smart_Update_Min(&front_left,distance);
		else if(Smart_Angle_In_Range(angle,SMART_RIGHT_MIN_DEG,SMART_RIGHT_MAX_DEG))
			Smart_Update_Min(&front_right,distance);
	}

	SmartCruise_Sectors.front_mm = Smart_Finalize_Min(front);
	SmartCruise_Sectors.front_left_mm = Smart_Finalize_Min(front_left);
	SmartCruise_Sectors.front_right_mm = Smart_Finalize_Min(front_right);
	SmartCruise_Sectors.left_mm = SmartCruise_Sectors.front_left_mm;
	SmartCruise_Sectors.right_mm = SmartCruise_Sectors.front_right_mm;
	SmartCruise_Sectors.left_rear_mm = 0;
	SmartCruise_Sectors.right_rear_mm = 0;
	SmartCruise_Sectors.rear_mm = 0;
	SmartCruise_Sectors.valid = valid;

	SmartCruise_Telemetry.nearest_mm = Smart_Finalize_Min(nearest);
	SmartCruise_Telemetry.nearest_angle = nearest_angle;
	SmartCruise_Debug.front_offset_deg = Smart_Front_Angle_Offset;
	SmartCruise_Debug.front_min_deg = SMART_FRONT_MIN_DEG;
	SmartCruise_Debug.front_max_deg = SMART_FRONT_MAX_DEG;
	SmartCruise_Debug.front_hit_mm = Smart_Finalize_Min(front_hit);
	SmartCruise_Debug.front_hit_angle_deg = front_hit_angle;
	SmartCruise_Debug.front_point_count = front_count;
	SmartCruise_Debug.nearest_raw_angle_deg = nearest_raw_angle;
	SmartCruise_Debug.nearest_body_angle_deg = nearest_angle;

	Watch_Front_Offset_Deg = Smart_Front_Angle_Offset;
	Watch_Front_Hit_Angle_Deg = front_hit_angle;
	Watch_Nearest_Raw_Angle_Deg = nearest_raw_angle;
	Watch_Nearest_Body_Angle_Deg = nearest_angle;
	Watch_Front_Offset_X10 = Smart_Angle_To_X10(Smart_Front_Angle_Offset);
	Watch_Front_Hit_Angle_X10 = Smart_Angle_To_X10(front_hit_angle);
	Watch_Nearest_Raw_Angle_X10 = Smart_Angle_To_X10(nearest_raw_angle);
	Watch_Nearest_Body_Angle_X10 = Smart_Angle_To_X10(nearest_angle);
	Watch_Front_Hit_MM = Smart_Finalize_Min(front_hit);
	Watch_Front_Point_Count = front_count;
	Watch_Front_Window_Min_X10 = Smart_Angle_To_X10(SMART_FRONT_MIN_DEG);
	Watch_Front_Window_Max_X10 = Smart_Angle_To_X10(SMART_FRONT_MAX_DEG);
}

void Smart_Cruise_Task(void)
{
	Smart_Cruise_UpdateSectors();
	if(SmartCruise_Sectors.valid)
		SmartCruise_Telemetry.no_data_ticks = 0;
	else if(SmartCruise_Telemetry.no_data_ticks < 60000)
		SmartCruise_Telemetry.no_data_ticks++;

	if(SmartCruise_Telemetry.no_data_ticks >
		SmartCruise_Config.no_data_timeout_ticks)
	{
		SmartCruise_Telemetry.state = SMART_STATE_PAUSE_STOP;
		Move_X = 0;
		Move_Z = 0;
		return;
	}

	switch(SmartCruise_Telemetry.state)
	{
		case SMART_STATE_CRUISE_FORWARD:
			Move_X = SmartCruise_Config.cruise_speed;
			Move_Z = 0;
			state_tick = 0;
			clear_tick = 0;
			turn_direction = 0;
			SmartCruise_Telemetry.turn_dir = 0;
			if(Smart_Front_Close())
			{
				SmartCruise_Telemetry.obstacle_count++;
				if(Smart_Surrounded())
				{
					turn_direction = Smart_Select_Turn_Direction();
					surround_escape_active = 1;
					SmartCruise_Telemetry.state = SMART_STATE_ESCAPE_BACK;
				}
				else
				{
					Smart_Set_Turn_Command();
					SmartCruise_Telemetry.state = SMART_STATE_AVOID_TURN;
				}
			}
			break;

		case SMART_STATE_AVOID_SLOW:
		case SMART_STATE_AVOID_TURN:
			SmartCruise_Telemetry.state = SMART_STATE_AVOID_TURN;
			Smart_Set_Turn_Command();
			if(Smart_Surrounded())
			{
				state_tick = 0;
				clear_tick = 0;
				surround_escape_active = 1;
				SmartCruise_Telemetry.state = SMART_STATE_ESCAPE_BACK;
			}
			else if(!Smart_Front_Close() && Smart_Front_Clear() &&
				state_tick >= SMART_MIN_TURN_TICKS)
			{
				if(++clear_tick > SMART_CLEAR_CONFIRM_TICKS)
				{
					state_tick = 0;
					clear_tick = 0;
					turn_direction = 0;
					surround_escape_active = 0;
					SmartCruise_Telemetry.turn_dir = 0;
					SmartCruise_Telemetry.state = SMART_STATE_CRUISE_FORWARD;
				}
			}
			else
				clear_tick = 0;

			if(SmartCruise_Telemetry.state == SMART_STATE_AVOID_TURN &&
				++state_tick > SMART_AVOID_TIMEOUT_TICKS)
			{
				state_tick = 0;
				clear_tick = 0;
				SmartCruise_Telemetry.state = SMART_STATE_ESCAPE_BACK;
			}
			break;

		case SMART_STATE_ESCAPE_BACK:
			Move_X = SmartCruise_Config.back_speed;
			Move_Z = 0;
			SmartCruise_Telemetry.turn_dir = 0;
			if(++state_tick > SMART_ESCAPE_BACK_TICKS)
			{
				state_tick = 0;
				clear_tick = 0;
				SmartCruise_Telemetry.state = surround_escape_active ?
					SMART_STATE_ESCAPE_TURN : SMART_STATE_AVOID_TURN;
			}
			break;

		case SMART_STATE_ESCAPE_TURN:
			Move_X = SmartCruise_Config.back_speed;
			Move_Z = (turn_direction == SMART_TURN_RIGHT) ?
				SmartCruise_Config.max_turn_angle :
				-SmartCruise_Config.max_turn_angle;
			SmartCruise_Telemetry.turn_dir =
				(turn_direction == SMART_TURN_LEFT) ? 1 : 2;
			if(++state_tick > SMART_ESCAPE_TURN_TICKS)
			{
				state_tick = 0;
				clear_tick = 0;
				surround_escape_active = 0;
				SmartCruise_Telemetry.state = SMART_STATE_AVOID_TURN;
			}
			break;

		case SMART_STATE_PAUSE_STOP:
		default:
			Move_X = 0;
			Move_Z = 0;
			if(SmartCruise_Sectors.valid)
			{
				state_tick = 0;
				clear_tick = 0;
				turn_direction = 0;
				surround_escape_active = 0;
				SmartCruise_Telemetry.turn_dir = 0;
				SmartCruise_Telemetry.state = SMART_STATE_CRUISE_FORWARD;
			}
			break;
	}
}
