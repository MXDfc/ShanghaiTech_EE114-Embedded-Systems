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
	0.12f,   /* slow_speed */
	-0.14f,  /* back_speed */
	0.38f,   /* max_turn_angle, rad for Ackermann front wheel */
	850,     /* warn_distance_mm */
	520,     /* stop_distance_mm */
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
#define SMART_TURN_LOCK_TICKS         260
#define SMART_TURN_SWITCH_MARGIN_MM  1200
#define SMART_CLEAR_CONFIRM_TICKS      36
#define SMART_CLEAR_FRONT_MARGIN_MM   150
#define SMART_CLEAR_SIDE_MARGIN_MM    180
#define SMART_SIDE_WARN_DISTANCE_MM   430
#define SMART_EXIT_CONFIRM_TICKS       18
#define SMART_EXIT_STRAIGHT_TICKS      90
#define SMART_LIDAR_MIN_VALID_MM       30
#define SMART_LIDAR_NEAR_CLAMP_MM      80
#define SMART_LIDAR_MAX_VALID_MM     5000

static u16 state_tick = 0;
static u16 turn_lock_tick = 0;
static u16 clear_tick = 0;
static int turn_direction = 0;

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
	return (distance == 0) ? 5000 : distance;
}

static u16 Smart_Direction_Score(int direction)
{
	u16 front_space,side_space,rear_side_space;

	if(direction == SMART_TURN_LEFT)
	{
		front_space = Smart_Open_Space(SmartCruise_Sectors.front_left_mm);
		side_space = Smart_Open_Space(SmartCruise_Sectors.left_mm);
		rear_side_space = Smart_Open_Space(SmartCruise_Sectors.left_rear_mm);
	}
	else
	{
		front_space = Smart_Open_Space(SmartCruise_Sectors.front_right_mm);
		side_space = Smart_Open_Space(SmartCruise_Sectors.right_mm);
		rear_side_space = Smart_Open_Space(SmartCruise_Sectors.right_rear_mm);
	}

	return front_space + side_space / 2 + rear_side_space / 4;
}

static int Smart_Select_Desired_Direction(void)
{
	u16 left_score = Smart_Direction_Score(SMART_TURN_LEFT);
	u16 right_score = Smart_Direction_Score(SMART_TURN_RIGHT);

	return (left_score >= right_score) ? SMART_TURN_LEFT : SMART_TURN_RIGHT;
}

static float Smart_Select_Turn(u8 allow_switch)
{
	int desired_direction = Smart_Select_Desired_Direction();

	if(turn_direction == 0)
	{
		turn_direction = desired_direction;
		turn_lock_tick = 0;
	}
	else if(allow_switch &&
		turn_lock_tick >= SMART_TURN_LOCK_TICKS &&
		desired_direction != turn_direction &&
		Smart_Direction_Score(desired_direction) >
		(u16)(Smart_Direction_Score(turn_direction) + SMART_TURN_SWITCH_MARGIN_MM))
	{
		turn_direction = desired_direction;
		turn_lock_tick = 0;
	}

	if(turn_lock_tick < 60000)
		turn_lock_tick++;
	SmartCruise_Telemetry.turn_dir = (turn_direction == SMART_TURN_LEFT) ? 1 : 2;

	return (turn_direction == SMART_TURN_LEFT) ?
		SmartCruise_Config.max_turn_angle : -SmartCruise_Config.max_turn_angle;
}

static u8 Smart_Distance_Clear(u16 distance,u16 threshold)
{
	return (distance == 0 || distance > threshold);
}

static u8 Smart_Path_Clear(void)
{
	return Smart_Distance_Clear(SmartCruise_Sectors.front_mm,
			SmartCruise_Config.warn_distance_mm + SMART_CLEAR_FRONT_MARGIN_MM) &&
		Smart_Distance_Clear(SmartCruise_Sectors.front_left_mm,
			SmartCruise_Config.stop_distance_mm + SMART_CLEAR_SIDE_MARGIN_MM) &&
		Smart_Distance_Clear(SmartCruise_Sectors.front_right_mm,
			SmartCruise_Config.stop_distance_mm + SMART_CLEAR_SIDE_MARGIN_MM);
}

static u8 Smart_Locked_Exit_Clear(void)
{
	if(turn_direction == SMART_TURN_LEFT)
	{
		return Smart_Distance_Clear(SmartCruise_Sectors.front_mm,
				SmartCruise_Config.stop_distance_mm + SMART_CLEAR_FRONT_MARGIN_MM) &&
			Smart_Distance_Clear(SmartCruise_Sectors.front_left_mm,
				SmartCruise_Config.warn_distance_mm);
	}
	else if(turn_direction == SMART_TURN_RIGHT)
	{
		return Smart_Distance_Clear(SmartCruise_Sectors.front_mm,
				SmartCruise_Config.stop_distance_mm + SMART_CLEAR_FRONT_MARGIN_MM) &&
			Smart_Distance_Clear(SmartCruise_Sectors.front_right_mm,
				SmartCruise_Config.warn_distance_mm);
	}

	return 0;
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
	turn_lock_tick = 0;
	clear_tick = 0;
	turn_direction = 0;
	Move_X = 0;
	Move_Z = 0;
}

void Smart_Cruise_UpdateSectors(void)
{
	int i,count;
	u16 distance;
	float angle;
	u16 front = 65535;
	u16 front_left = 65535;
	u16 front_right = 65535;
	u16 left = 65535;
	u16 right = 65535;
	u16 left_rear = 65535;
	u16 right_rear = 65535;
	u16 rear = 65535;
	u16 nearest = 65535;
	float nearest_angle = 0.0f;
	float raw_angle;
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
		if(distance < SMART_LIDAR_MIN_VALID_MM || distance > SMART_LIDAR_MAX_VALID_MM)
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
			SmartCruise_Debug.nearest_raw_angle_deg = raw_angle;
		}

		if(Smart_Angle_In_Range(angle,-28.0f,28.0f))
		{
			Smart_Update_Min(&front,distance);
			front_count++;
			if(distance < front_hit)
			{
				front_hit = distance;
				front_hit_angle = angle;
			}
		}
		if(Smart_Angle_In_Range(angle,15.0f,70.0f))
			Smart_Update_Min(&front_left,distance);
		if(Smart_Angle_In_Range(angle,-70.0f,-15.0f))
			Smart_Update_Min(&front_right,distance);
		if(Smart_Angle_In_Range(angle,70.0f,125.0f))
			Smart_Update_Min(&left,distance);
		if(Smart_Angle_In_Range(angle,-125.0f,-70.0f))
			Smart_Update_Min(&right,distance);
		if(Smart_Angle_In_Range(angle,125.0f,150.0f))
			Smart_Update_Min(&left_rear,distance);
		if(Smart_Angle_In_Range(angle,-150.0f,-125.0f))
			Smart_Update_Min(&right_rear,distance);
		if((angle >= 150.0f) || (angle <= -150.0f))
			Smart_Update_Min(&rear,distance);
	}

	SmartCruise_Sectors.front_mm = Smart_Finalize_Min(front);
	SmartCruise_Sectors.front_left_mm = Smart_Finalize_Min(front_left);
	SmartCruise_Sectors.front_right_mm = Smart_Finalize_Min(front_right);
	SmartCruise_Sectors.left_mm = Smart_Finalize_Min(left);
	SmartCruise_Sectors.right_mm = Smart_Finalize_Min(right);
	SmartCruise_Sectors.left_rear_mm = Smart_Finalize_Min(left_rear);
	SmartCruise_Sectors.right_rear_mm = Smart_Finalize_Min(right_rear);
	SmartCruise_Sectors.rear_mm = Smart_Finalize_Min(rear);
	SmartCruise_Sectors.valid = valid;
	SmartCruise_Telemetry.nearest_mm = Smart_Finalize_Min(nearest);
	SmartCruise_Telemetry.nearest_angle = nearest_angle;
	SmartCruise_Debug.front_offset_deg = Smart_Front_Angle_Offset;
	SmartCruise_Debug.front_min_deg = -28.0f;
	SmartCruise_Debug.front_max_deg = 28.0f;
	SmartCruise_Debug.front_hit_mm = Smart_Finalize_Min(front_hit);
	SmartCruise_Debug.front_hit_angle_deg = front_hit_angle;
	SmartCruise_Debug.front_point_count = front_count;
	SmartCruise_Debug.nearest_body_angle_deg = nearest_angle;
	Watch_Front_Offset_Deg = Smart_Front_Angle_Offset;
	Watch_Front_Hit_Angle_Deg = front_hit_angle;
	Watch_Nearest_Raw_Angle_Deg = SmartCruise_Debug.nearest_raw_angle_deg;
	Watch_Nearest_Body_Angle_Deg = nearest_angle;
	Watch_Front_Offset_X10 = Smart_Angle_To_X10(Smart_Front_Angle_Offset);
	Watch_Front_Hit_Angle_X10 = Smart_Angle_To_X10(front_hit_angle);
	Watch_Nearest_Raw_Angle_X10 = Smart_Angle_To_X10(SmartCruise_Debug.nearest_raw_angle_deg);
	Watch_Nearest_Body_Angle_X10 = Smart_Angle_To_X10(nearest_angle);
	Watch_Front_Hit_MM = Smart_Finalize_Min(front_hit);
	Watch_Front_Point_Count = front_count;
	Watch_Front_Window_Min_X10 = -280;
	Watch_Front_Window_Max_X10 = 280;
}

void Smart_Cruise_Task(void)
{
	u8 front_blocked, front_warning, escape_needed;
	float turn;

	Smart_Cruise_UpdateSectors();
	if(SmartCruise_Sectors.valid)
		SmartCruise_Telemetry.no_data_ticks = 0;
	else if(SmartCruise_Telemetry.no_data_ticks < 60000)
		SmartCruise_Telemetry.no_data_ticks++;

	if(SmartCruise_Telemetry.no_data_ticks > SmartCruise_Config.no_data_timeout_ticks)
	{
		SmartCruise_Telemetry.state = SMART_STATE_PAUSE_STOP;
		Move_X = 0;
		Move_Z = 0;
		return;
	}

	front_blocked = (SmartCruise_Sectors.front_mm > 0 &&
		SmartCruise_Sectors.front_mm < SmartCruise_Config.stop_distance_mm);
	front_warning = (SmartCruise_Sectors.front_mm > 0 &&
		SmartCruise_Sectors.front_mm < SmartCruise_Config.warn_distance_mm) ||
		(SmartCruise_Sectors.front_left_mm > 0 &&
		SmartCruise_Sectors.front_left_mm < SmartCruise_Config.stop_distance_mm) ||
		(SmartCruise_Sectors.front_right_mm > 0 &&
		SmartCruise_Sectors.front_right_mm < SmartCruise_Config.stop_distance_mm);
	escape_needed = (SmartCruise_Sectors.front_mm > 0 &&
		SmartCruise_Sectors.front_mm < SmartCruise_Config.escape_distance_mm);

	switch(SmartCruise_Telemetry.state)
	{
		case SMART_STATE_CRUISE_FORWARD:
			Move_X = SmartCruise_Config.cruise_speed;
			Move_Z = 0;
			state_tick = 0;
			clear_tick = 0;
			turn_direction = 0;
			turn_lock_tick = 0;
			SmartCruise_Telemetry.turn_dir = 0;
			if(front_warning)
			{
				SmartCruise_Telemetry.obstacle_count++;
				SmartCruise_Telemetry.state = front_blocked ? SMART_STATE_AVOID_TURN : SMART_STATE_AVOID_SLOW;
			}
			break;

		case SMART_STATE_AVOID_SLOW:
			Move_X = SmartCruise_Config.slow_speed;
			turn = Smart_Select_Turn(0);
			Move_Z = turn;
			if(front_blocked)
			{
				state_tick = 0;
				SmartCruise_Telemetry.state = SMART_STATE_AVOID_TURN;
			}
			else if(Smart_Locked_Exit_Clear())
			{
				if(++clear_tick > SMART_EXIT_CONFIRM_TICKS)
				{
					clear_tick = 0;
					state_tick = 0;
					SmartCruise_Telemetry.state = SMART_STATE_EXIT_STRAIGHT;
				}
			}
			else if(Smart_Path_Clear())
			{
				if(++clear_tick > SMART_CLEAR_CONFIRM_TICKS)
				{
					clear_tick = 0;
					SmartCruise_Telemetry.state = SMART_STATE_CRUISE_FORWARD;
				}
			}
			else
				clear_tick = 0;
			break;

		case SMART_STATE_EXIT_STRAIGHT:
			Move_X = SmartCruise_Config.slow_speed;
			Move_Z = 0;
			if(front_blocked)
			{
				state_tick = 0;
				clear_tick = 0;
				SmartCruise_Telemetry.state = SMART_STATE_AVOID_TURN;
			}
			else if(++state_tick > SMART_EXIT_STRAIGHT_TICKS)
			{
				state_tick = 0;
				clear_tick = 0;
				turn_direction = 0;
				turn_lock_tick = 0;
				SmartCruise_Telemetry.turn_dir = 0;
				SmartCruise_Telemetry.state = SMART_STATE_CRUISE_FORWARD;
			}
			break;

		case SMART_STATE_AVOID_TURN:
			Move_X = 0.06f;
			turn = Smart_Select_Turn(0);
			Move_Z = turn;
			if(escape_needed || ++state_tick > 180)
			{
				state_tick = 0;
				SmartCruise_Telemetry.state = SMART_STATE_ESCAPE_BACK;
			}
			else if(!front_blocked)
			{
				state_tick = 0;
				clear_tick = 0;
				SmartCruise_Telemetry.state = SMART_STATE_AVOID_SLOW;
			}
			break;

		case SMART_STATE_ESCAPE_BACK:
			Move_X = SmartCruise_Config.back_speed;
			Move_Z = -Smart_Select_Turn(0) * 0.8f;
			if(++state_tick > 90)
			{
				state_tick = 0;
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
				SmartCruise_Telemetry.state = SMART_STATE_CRUISE_FORWARD;
			}
			break;
	}
}
