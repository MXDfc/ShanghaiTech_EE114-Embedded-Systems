/***********************************************
Lightweight local mapping navigation for EE114 project.
***********************************************/

#include "lidar_mapping_nav.h"
#include "Header.h"
#include "Lidar.h"

#if LD14P
extern LidarPointStructDef Dataprocess[800];
#else
extern PointDataProcessDef Dataprocess[400];
#endif

MapNavPose MapNav_Pose;
MapNavTelemetry MapNav_Telemetry;

#define MAP_NAV_SIZE                 80
#define MAP_NAV_CENTER               (MAP_NAV_SIZE / 2)
#define MAP_NAV_RES_MM               50
#define MAP_NAV_VALID_MIN_MM         80
#define MAP_NAV_VALID_MAX_MM       3500
#define MAP_NAV_OCC_INC              12
#define MAP_NAV_FREE_DEC              4
#define MAP_NAV_OCC_TH               24
#define MAP_NAV_CLEAR_TH              8
#define MAP_NAV_RAY_STEP_MM         100
#define MAP_NAV_FORWARD_SPEED      0.22f
#define MAP_NAV_SLOW_SPEED         0.12f
#define MAP_NAV_BACK_SPEED        -0.14f
#define MAP_NAV_MAX_STEER          0.38f
#define MAP_NAV_FRONT_BLOCK_MM      380
#define MAP_NAV_PATH_CHECK_MM      1200
#define MAP_NAV_BODY_RADIUS_MM      170
#define MAP_NAV_NO_DATA_TICKS        80
#define MAP_NAV_BACK_TICKS          100
#define MAP_NAV_RECOVER_TICKS       120
#define MAP_NAV_SCORE_BLOCKED    -30000L

static signed char occ_grid[MAP_NAV_SIZE][MAP_NAV_SIZE];
static u16 state_tick = 0;
static float locked_angle_deg = 0.0f;

static const float candidate_angle_deg[] = {
	-75.0f,-60.0f,-45.0f,-30.0f,-15.0f,0.0f,
	15.0f,30.0f,45.0f,60.0f,75.0f
};

static float MapNav_Normalize_Angle(float angle)
{
	while(angle > 180.0f) angle -= 360.0f;
	while(angle < -180.0f) angle += 360.0f;
	return angle;
}

static float MapNav_Normalize_Rad(float angle)
{
	while(angle > Pi) angle -= 2.0f * Pi;
	while(angle < -Pi) angle += 2.0f * Pi;
	return angle;
}

static int MapNav_Clamp_Int(int value,int min_value,int max_value)
{
	if(value < min_value) return min_value;
	if(value > max_value) return max_value;
	return value;
}

static void MapNav_Add_Cell(int gx,int gy,int delta)
{
	int value;

	if(gx < 0 || gx >= MAP_NAV_SIZE || gy < 0 || gy >= MAP_NAV_SIZE)
		return;

	value = occ_grid[gy][gx] + delta;
	value = MapNav_Clamp_Int(value,-127,127);
	occ_grid[gy][gx] = (signed char)value;
}

static u8 MapNav_World_To_Grid(float x_mm,float y_mm,int *gx,int *gy)
{
	int ix = MAP_NAV_CENTER + (int)(x_mm / MAP_NAV_RES_MM);
	int iy = MAP_NAV_CENTER - (int)(y_mm / MAP_NAV_RES_MM);

	if(ix < 0 || ix >= MAP_NAV_SIZE || iy < 0 || iy >= MAP_NAV_SIZE)
		return 0;

	*gx = ix;
	*gy = iy;
	return 1;
}

static signed char MapNav_Cell_Cost_By_World(float x_mm,float y_mm)
{
	int gx,gy;

	if(!MapNav_World_To_Grid(x_mm,y_mm,&gx,&gy))
		return MAP_NAV_OCC_TH;

	return occ_grid[gy][gx];
}

static void MapNav_Decay_Map(void)
{
	int x,y;

	for(y=0;y<MAP_NAV_SIZE;y++)
	{
		for(x=0;x<MAP_NAV_SIZE;x++)
		{
			if(occ_grid[y][x] > 0)
				occ_grid[y][x]--;
			else if(occ_grid[y][x] < 0)
				occ_grid[y][x]++;
		}
	}
}

static void MapNav_Update_Odometry(void)
{
	float speed = (MotorA.Current_Encoder + MotorB.Current_Encoder) * 0.5f;
	float dt = 1.0f / Frequency;
	float ds_mm = speed * dt * 1000.0f;
	float steer = Move_Z;
	float dtheta = 0.0f;

	if(Car_Num == Akm_Car)
		dtheta = speed * tan(steer) / Akm_axlespacing * dt;
	else
		dtheta = (MotorB.Current_Encoder - MotorA.Current_Encoder) /
			Wheelspacing * dt;

	MapNav_Pose.theta_rad = MapNav_Normalize_Rad(MapNav_Pose.theta_rad + dtheta);
	MapNav_Pose.x_mm += ds_mm * cos(MapNav_Pose.theta_rad);
	MapNav_Pose.y_mm += ds_mm * sin(MapNav_Pose.theta_rad);
}

static u8 MapNav_Point_Valid(u16 distance)
{
	return (distance >= MAP_NAV_VALID_MIN_MM &&
		distance <= MAP_NAV_VALID_MAX_MM);
}

static void MapNav_Update_Ray(float body_angle_rad,u16 distance)
{
	int step;
	int gx,gy;
	float ray_x,ray_y;
	float world_angle = MapNav_Pose.theta_rad + body_angle_rad;
	u16 free_distance = (distance > MAP_NAV_RAY_STEP_MM) ?
		(distance - MAP_NAV_RAY_STEP_MM) : 0;

	for(step=MAP_NAV_RAY_STEP_MM;step<free_distance;step+=MAP_NAV_RAY_STEP_MM)
	{
		ray_x = MapNav_Pose.x_mm + (float)step * cos(world_angle);
		ray_y = MapNav_Pose.y_mm + (float)step * sin(world_angle);
		if(MapNav_World_To_Grid(ray_x,ray_y,&gx,&gy))
			MapNav_Add_Cell(gx,gy,-MAP_NAV_FREE_DEC);
	}

	ray_x = MapNav_Pose.x_mm + (float)distance * cos(world_angle);
	ray_y = MapNav_Pose.y_mm + (float)distance * sin(world_angle);
	if(MapNav_World_To_Grid(ray_x,ray_y,&gx,&gy))
		MapNav_Add_Cell(gx,gy,MAP_NAV_OCC_INC);
}

static void MapNav_Update_Map(void)
{
	int i,count;
	u16 distance;
	float angle_rad;
	u16 nearest = 65535;
	float nearest_angle = 0.0f;
	u8 valid = 0;
	u8 occupied = 0;

	count = lap_count;
#if LD14P
	if(count > 800) count = 800;
#else
	if(count > 400) count = 400;
#endif

	MapNav_Decay_Map();

	for(i=0;i<count;i++)
	{
		distance = Dataprocess[i].distance;
		if(!MapNav_Point_Valid(distance))
			continue;

		valid = 1;
		angle_rad = MapNav_Normalize_Angle(Dataprocess[i].angle) / Angle_To_Rad;
		MapNav_Update_Ray(angle_rad,distance);

		if(distance < nearest)
		{
			nearest = distance;
			nearest_angle = MapNav_Normalize_Angle(Dataprocess[i].angle);
		}
	}

	for(i=0;i<MAP_NAV_SIZE * MAP_NAV_SIZE;i++)
	{
		if(((signed char *)occ_grid)[i] > MAP_NAV_OCC_TH)
			occupied++;
	}

	if(valid)
	{
		MapNav_Telemetry.no_data_ticks = 0;
		MapNav_Telemetry.map_updates++;
	}
	else if(MapNav_Telemetry.no_data_ticks < 60000)
		MapNav_Telemetry.no_data_ticks++;

	MapNav_Telemetry.nearest_mm = (nearest == 65535) ? 0 : nearest;
	MapNav_Telemetry.nearest_angle_deg = nearest_angle;
	MapNav_Telemetry.occupied_cells = occupied;
}

static long MapNav_Score_Path(float angle_deg,u16 *clear_mm)
{
	int d,side;
	float angle_rad = MapNav_Pose.theta_rad + angle_deg / Angle_To_Rad;
	float x,y;
	signed char cost;
	long score = 0;

	*clear_mm = MAP_NAV_PATH_CHECK_MM;
	for(d=150;d<=MAP_NAV_PATH_CHECK_MM;d+=MAP_NAV_RES_MM)
	{
		for(side=-MAP_NAV_BODY_RADIUS_MM;side<=MAP_NAV_BODY_RADIUS_MM;
			side+=MAP_NAV_RES_MM)
		{
			x = MapNav_Pose.x_mm + (float)d * cos(angle_rad) -
				(float)side * sin(angle_rad);
			y = MapNav_Pose.y_mm + (float)d * sin(angle_rad) +
				(float)side * cos(angle_rad);
			cost = MapNav_Cell_Cost_By_World(x,y);
			if(cost > MAP_NAV_OCC_TH)
			{
				*clear_mm = (u16)d;
				return MAP_NAV_SCORE_BLOCKED + d;
			}
			else if(cost > MAP_NAV_CLEAR_TH)
				score -= cost * 8;
			else if(cost < -MAP_NAV_CLEAR_TH)
				score += 10;
		}
	}

	score += (long)(*clear_mm) * 10L;
	score -= (long)(float_abs(angle_deg) * 18.0f);
	score -= (long)(float_abs(angle_deg - locked_angle_deg) * 4.0f);
	return score;
}

static float MapNav_Select_Best_Angle(u16 *best_clear)
{
	u8 i;
	long score,best_score = -32000L;
	u16 clear_mm;
	float best_angle = 0.0f;

	for(i=0;i<sizeof(candidate_angle_deg)/sizeof(candidate_angle_deg[0]);i++)
	{
		score = MapNav_Score_Path(candidate_angle_deg[i],&clear_mm);
		if(score > best_score)
		{
			best_score = score;
			best_angle = candidate_angle_deg[i];
			*best_clear = clear_mm;
		}
	}

	return best_angle;
}

static float MapNav_Angle_To_Steer(float angle_deg)
{
	float steer = angle_deg / Angle_To_Rad * 0.75f;

	return target_limit_float(steer,-MAP_NAV_MAX_STEER,MAP_NAV_MAX_STEER);
}

void Lidar_Mapping_Nav_Reset(void)
{
	int x,y;

	for(y=0;y<MAP_NAV_SIZE;y++)
	{
		for(x=0;x<MAP_NAV_SIZE;x++)
			occ_grid[y][x] = 0;
	}

	MapNav_Pose.x_mm = 0.0f;
	MapNav_Pose.y_mm = 0.0f;
	MapNav_Pose.theta_rad = 0.0f;
	MapNav_Telemetry.state = MAP_NAV_STATE_CRUISE;
	MapNav_Telemetry.nearest_mm = 0;
	MapNav_Telemetry.nearest_angle_deg = 0.0f;
	MapNav_Telemetry.best_angle_deg = 0.0f;
	MapNav_Telemetry.best_clear_mm = 0;
	MapNav_Telemetry.map_updates = 0;
	MapNav_Telemetry.no_data_ticks = 0;
	MapNav_Telemetry.occupied_cells = 0;
	state_tick = 0;
	locked_angle_deg = 0.0f;
	Move_X = 0;
	Move_Z = 0;
}

void Lidar_Mapping_Nav_Task(void)
{
	u16 best_clear = 0;
	float best_angle;

	MapNav_Update_Odometry();
	MapNav_Update_Map();

	if(MapNav_Telemetry.no_data_ticks > MAP_NAV_NO_DATA_TICKS)
	{
		MapNav_Telemetry.state = MAP_NAV_STATE_PAUSE;
		Move_X = 0;
		Move_Z = 0;
		return;
	}

	best_angle = MapNav_Select_Best_Angle(&best_clear);
	MapNav_Telemetry.best_angle_deg = best_angle;
	MapNav_Telemetry.best_clear_mm = best_clear;

	switch(MapNav_Telemetry.state)
	{
		case MAP_NAV_STATE_CRUISE:
			locked_angle_deg = best_angle;
			Move_X = MAP_NAV_FORWARD_SPEED;
			Move_Z = MapNav_Angle_To_Steer(best_angle);
			if((MapNav_Telemetry.nearest_mm > 0 &&
				MapNav_Telemetry.nearest_mm < MAP_NAV_FRONT_BLOCK_MM) ||
				best_clear < MAP_NAV_FRONT_BLOCK_MM)
			{
				state_tick = 0;
				MapNav_Telemetry.state = MAP_NAV_STATE_AVOID;
			}
			break;

		case MAP_NAV_STATE_AVOID:
			if(best_clear < MAP_NAV_BODY_RADIUS_MM * 2)
			{
				state_tick = 0;
				MapNav_Telemetry.state = MAP_NAV_STATE_BACK;
			}
			else
			{
				locked_angle_deg = best_angle;
				Move_X = MAP_NAV_SLOW_SPEED;
				Move_Z = MapNav_Angle_To_Steer(best_angle);
				if(best_clear > MAP_NAV_PATH_CHECK_MM - 100 &&
					float_abs(best_angle) < 20.0f)
				{
					state_tick = 0;
					MapNav_Telemetry.state = MAP_NAV_STATE_CRUISE;
				}
			}
			break;

		case MAP_NAV_STATE_BACK:
			Move_X = MAP_NAV_BACK_SPEED;
			Move_Z = 0;
			if(++state_tick > MAP_NAV_BACK_TICKS)
			{
				state_tick = 0;
				MapNav_Telemetry.state = MAP_NAV_STATE_RECOVER_TURN;
			}
			break;

		case MAP_NAV_STATE_RECOVER_TURN:
			Move_X = MAP_NAV_BACK_SPEED;
			Move_Z = (locked_angle_deg >= 0.0f) ?
				-MAP_NAV_MAX_STEER : MAP_NAV_MAX_STEER;
			if(++state_tick > MAP_NAV_RECOVER_TICKS)
			{
				state_tick = 0;
				MapNav_Telemetry.state = MAP_NAV_STATE_AVOID;
			}
			break;

		case MAP_NAV_STATE_PAUSE:
		default:
			Move_X = 0;
			Move_Z = 0;
			if(MapNav_Telemetry.no_data_ticks == 0)
			{
				state_tick = 0;
				MapNav_Telemetry.state = MAP_NAV_STATE_CRUISE;
			}
			break;
	}
}
