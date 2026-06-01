/***********************************************
???????????????
???WHEELTEC
???wheeltec.net
?????shop114407458.taobao.com
???: https://minibalance.aliexpress.com/store/4455017
???V1.0
?????2023-03-02

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update?2023-03-02

All rights reserved
***********************************************/


#ifndef __CONTROL_H
#define __CONTROL_H

#include "Lidar.h"
#include "Header.h"

//PWM???????
#define PWM_MAX  6900
#define PWM_MIN  -6900


#define Default_Velocity					350			//?????????mm/s
#define Default_Turn_Bias					73			//??????????/s
#define Bluetooth_Turn_Angle				Pi/5		//???????????????rad
#define PS2_Turn_Angle						Pi/4		//???????????????rad

#define forward_velocity 0.25  //Move_X??

#define Follow_Distance 1200  //????
#define Keep_Follow_Distance 400  //??????

#define Aovid_Speed 0.3        //????

#define FILTERING_TIMES 					10			//????

//???????
#define Normal_Mode							0
#define Lidar_Avoid_Mode					1
#define Lidar_Follow_Mode					2
#define Lidar_Along_Mode				    3		
#define ELE_Line_Patrol_Mode				4
#define CCD_Line_Patrol_Mode				5
#define ROS_Mode				            6
#define Measure_Distance_Mode				7			//??????

// ????????
#define AUTO_DEMO_IDLE                       0
#define AUTO_DEMO_FORWARD                    1
#define AUTO_DEMO_WAIT_AFTER_FORWARD         2
#define AUTO_DEMO_STEER_LEFT                 3
#define AUTO_DEMO_FORWARD_LEFT               4
#define AUTO_DEMO_WAIT_AFTER_FORWARD_LEFT    5
#define AUTO_DEMO_BACKWARD_LEFT              6
#define AUTO_DEMO_WAIT_AFTER_BACKWARD_LEFT   7
#define AUTO_DEMO_STEER_CENTER               8
#define AUTO_DEMO_WAIT_AFTER_CENTER          9
#define AUTO_DEMO_BACKWARD_FINAL             10
#define AUTO_DEMO_FINISHED                   11

// Final exhibition auto scan states. The old Auto Demo states above are kept for reference.
#define AUTO_SCAN_PREPARE                    20
#define AUTO_SCAN_LAP_MOVE                   21
#define AUTO_SCAN_LAP_SETTLE                 22
#define AUTO_SCAN_LAP_SAMPLE                 23
#define AUTO_SCAN_TURN_MOVE                  24
#define AUTO_SCAN_TURN_SETTLE                25
#define AUTO_SCAN_DRIVE_BEST                 26
#define AUTO_SCAN_FINISHED                   27


//?????????
#define RC_ON								1	
#define RC_OFF								0
//???????????
#define MAX_RC_Velocity						800
//???????????
#define	MAX_RC_Turn_Bias					1
//???????????
#define MINI_RC_Velocity					210
//???????????
#define	MINI_RC_Turn_Velocity				Pi/20

//???????????????????
#define X_Step								25
//????????
#define Z_Step								0.1

//????
#define Diff_Car_Wheel_diameter				0.0670f			//???????????mm
#define Small_Tank_WheelDiameter			0.0430f			//????
#define Big_Tank_WheelDiameter				0.0440f			//????

//????
#define Diff_wheelspacing					0.164f			//?????
#define Akm_wheelspacing					0.164f			//??????
#define Small_Tank_wheelspacing				0.135f			//??????
#define Big_Tank_wheelspacing				0.233f			//??????
#define Akm_axlespacing           			0.144f			//??????
#define Diff_axlespacing                    0.155f           //?????


#define Gear_Ratio							30.0f			//??????


#define Pi									3.1415926523f	//???
#define Angle_To_Rad						57.295779513f	//??????????????
#define Frequency							200.0f			//?5ms?????????
#define SERVO_INIT 							1500  			//????PWM?


#define Encoder_resolution_Photoelectric	500.0f			//?????500?
#define Encoder_resolution_Hall 			13.0f			//?????13?
#define Encoder_resolution 					Encoder_resolution_Hall		//??13??????

#define Angle_TO_PWM						640.62f			//????pwm??????

#define Normal								0				//???????0???
#define Abnormal							1


#define Lidar_Detect_ON						1				//???????????????
#define Lidar_Detect_OFF					0


//??????
typedef struct  
{
  int A;      
  int B;  
}Encoder;

//?????????????
typedef struct  
{
	float Current_Encoder;     	//??????????????
	float Motor_Pwm;     		//??PWM???????????
	float Target_Encoder;  		//???????????????????
	float Velocity; 	 		//?????
}Motor_parameter;




extern float Move_X,Move_Z;
extern float PWM_Left,PWM_Right;
extern float RC_Velocity,RC_Turn_Velocity;
extern u8 Mode;
extern Motor_parameter MotorA,MotorB;	
extern int Servo_PWM;
extern u8 Lidar_Detect;
extern float CCD_Move_X,ELE_Move_X;	
extern u8 Ros_count;
extern short Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
extern u8 Auto_Demo_Active;
extern u8 Auto_Demo_State;
extern volatile u8 Auto_Scan_Debug_Valid;
extern volatile u16 Auto_Scan_Debug_Point;
extern volatile u16 Auto_Scan_Debug_Distance;
extern volatile u16 Auto_Scan_Debug_Sample_Count;
extern volatile u16 Auto_Scan_Debug_Best_Point;
extern volatile u16 Auto_Scan_Debug_Best_Distance;
void Bluetooth_Control(void);
void PS2_Control(void);
void Get_Target_Encoder(float Vx,float Vz);
void Get_Motor_PWM(void);
void Get_Velocity_From_Encoder(void);

float PWM_Limit(float IN,int max,int min);
u8 Turn_Off(void);
float Mean_Filter_Left(float data);
float Mean_Filter_Right(float data);
void Lidar_Avoid(void);
void Lidar_Follow(void);
float Merge_Angle(float angle1,float angle2);
//float Get_Merge_Angle(float angle,PointDataProcessDef Data,u8 Current_Count);
void Ultrasonic_Follow(void);
void Car_Perimeter_Init(void);
void Lidar_along_wall(void);
void Get_Target_Encoder(float Vx,float Vz);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
void Get_Angle(u8 way);
void Remote_Control(void);
void Auto_Demo_Start(void);
void Auto_Demo_Stop(void);
void Auto_Demo_Update_5ms(void);
void Auto_Demo_Service_Main(void);
const char *Auto_Demo_State_Name(void);
#endif
