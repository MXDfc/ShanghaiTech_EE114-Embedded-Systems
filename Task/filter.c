#include "filter.h"

/**************************************************************************
Function: Simple Kalman filter for X axis
Input   : acceleration angle, angular velocity
Output  : filtered angle
**************************************************************************/
float dt=0.005;  // 5ms filter period
float Kalman_Filter_x(float Accel,float Gyro)
{
	static float angle_dot;
	static float angle;
	float Q_angle=0.001;
	float Q_gyro=0.003;
	float R_angle=0.5;
	char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
	angle+=(Gyro - Q_bias) * dt;
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0];

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;
	PP[0][1] += Pdot[1] * dt;
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;

	Angle_err = Accel - angle;

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;

	angle	+= K_0 * Angle_err;
	Q_bias	+= K_1 * Angle_err;
	angle_dot   = Gyro - Q_bias;
	return angle;
}

/**************************************************************************
Function: First order complementary filter for X axis
**************************************************************************/
float Complementary_Filter_x(float angle_m, float gyro_m)
{
	 static float angle;
	 float K1 =0.02;
   angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * dt);
	 return angle;
}

/**************************************************************************
Function: Simple Kalman filter for Y axis
**************************************************************************/
float Kalman_Filter_y(float Accel,float Gyro)
{
	static float angle_dot;
	static float angle;
	float Q_angle=0.001;
	float Q_gyro=0.003;
	float R_angle=0.5;
	char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
	angle+=(Gyro - Q_bias) * dt;
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0];
	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;
	PP[0][1] += Pdot[1] * dt;
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
	Angle_err = Accel - angle;

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;

	angle	+= K_0 * Angle_err;
	Q_bias	+= K_1 * Angle_err;
	angle_dot   = Gyro - Q_bias;
	return angle;
}

/**************************************************************************
Function: First order complementary filter for Y axis
**************************************************************************/
float Complementary_Filter_y(float angle_m, float gyro_m)
{
	 static float angle;
	 float K1 =0.02;
   angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * dt);
	 return angle;
}
