#include <stdint.h>
#include <math.h>
#include <stdio.h>

#include "mpu6050.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ioi2c.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "inv_mpu.h"

typedef uint8_t u8;
typedef uint16_t u16;

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define MOTION          (0)
#define NO_MOTION       (1)
#define DEFAULT_MPU_HZ  (200)
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
#define q30  1073741824.0f
short gyro[3], accel[3], sensors;
float Roll,Pitch,Yaw;
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;

/* ---- safe output snapshot ---- */
static MPU6050_Euler_t euler_snapshot = {0.0f, 0.0f, 0.0f};
static volatile uint8_t euler_ready = 0;  /* 1 after first valid DMP read */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}


static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static void run_self_test(void)
{
    int result;
    long gyro_st[3], accel_st[3];

    result = mpu_run_self_test(gyro_st, accel_st);
    printf("[MPU] self_test result=0x%X (expect 0x3 for MPU6050, no compass)\r\n", result);
    if ((result & 0x3) == 0x3) {
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro_st[0] = (long)(gyro_st[0] * sens);
        gyro_st[1] = (long)(gyro_st[1] * sens);
        gyro_st[2] = (long)(gyro_st[2] * sens);
        dmp_set_gyro_bias(gyro_st);
        mpu_get_accel_sens(&accel_sens);
        accel_st[0] *= accel_sens;
        accel_st[1] *= accel_sens;
        accel_st[2] *= accel_sens;
        dmp_set_accel_bias(accel_st);
        printf("[MPU] bias calibration applied\r\n");
    } else {
        printf("[MPU] !!! self_test FAILED, bias NOT set\r\n");
    }
}



uint8_t buffer[14];

int16_t  MPU6050_FIFO[6][11];
int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;



/**************************************************************************
Function: The new ADC data is updated to FIFO array for filtering
Input   : ax,ay,az: x,y,z axis acceleration data; gx,gy,gz: x,y,z axis angular velocity data
Output  : none
**************************************************************************/
void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
unsigned char i ;
int32_t sum=0;
for(i=1;i<10;i++){
MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
}
MPU6050_FIFO[0][9]=ax;
MPU6050_FIFO[1][9]=ay;
MPU6050_FIFO[2][9]=az;
MPU6050_FIFO[3][9]=gx;
MPU6050_FIFO[4][9]=gy;
MPU6050_FIFO[5][9]=gz;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[0][i];
}
MPU6050_FIFO[0][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[1][i];
}
MPU6050_FIFO[1][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[2][i];
}
MPU6050_FIFO[2][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[3][i];
}
MPU6050_FIFO[3][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[4][i];
}
MPU6050_FIFO[4][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[5][i];
}
MPU6050_FIFO[5][10]=sum/10;
}

/**************************************************************************
Function: Setting the clock source of mpu6050
**************************************************************************/
void MPU6050_setClockSource(uint8_t source){
    IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/**************************************************************************
Function: Set full-scale gyroscope range
**************************************************************************/
void MPU6050_setFullScaleGyroRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************************************************************
Function: Set full-scale accelerometer range
**************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************************************************************
Function: Set mpu6050 sleep mode
**************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************************************************************
Function: Read WHO_AM_I identity
**************************************************************************/
uint8_t MPU6050_getDeviceID(void) {
    IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
    return buffer[0];
}

/**************************************************************************
Function: Check whether mpu6050 is connected
**************************************************************************/
uint8_t MPU6050_testConnection(void) {
   if(MPU6050_getDeviceID() == 0x68)
   return 1;
   	else return 0;
}

/**************************************************************************
Function: Set I2C master mode
**************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************************************************************
Function: Set I2C bypass mode
**************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/**************************************************************************
Function: Initialize MPU6050
**************************************************************************/
void MPU6050_initialize(void) {
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_YGYRO);
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    MPU6050_setSleepEnabled(0);
	MPU6050_setI2CMasterModeEnabled(0);
	MPU6050_setI2CBypassEnabled(0);
}

/**************************************************************************
Function: DMP initialization
**************************************************************************/
void DMP_Init(void)
{
   int ret;
   u8 temp[1]={0};

   printf("[DMP] DMP_Init start\r\n");

   ret = i2cRead(0x68, 0x75, 1, temp);
   printf("[DMP] i2cRead WHO_AM_I: ret=%d, val=0x%02X (expect 0x68)\r\n", ret, temp[0]);
   if(temp[0]!=0x68) {
	   printf("[DMP] !!! WHO_AM_I mismatch, resetting!\r\n");
	   NVIC_SystemReset();
   }

   ret = mpu_init();
   printf("[DMP] mpu_init: ret=%d\r\n", ret);
   if(ret) {
       printf("[DMP] !!! mpu_init FAILED, aborting DMP\r\n");
       return;
   }

   /* self-test MUST run before DMP firmware load,
      because mpu_run_self_test internally resets the sensor */
   ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
   printf("[DMP] mpu_set_sensors: ret=%d\r\n", ret);

   run_self_test();

   /* After self-test the sensor may have been reconfigured,
      so re-do sensor/FIFO setup before loading DMP */
   ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
   printf("[DMP] mpu_set_sensors (post-selftest): ret=%d\r\n", ret);

   ret = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
   printf("[DMP] mpu_configure_fifo: ret=%d\r\n", ret);

   ret = mpu_set_sample_rate(DEFAULT_MPU_HZ);
   printf("[DMP] mpu_set_sample_rate(%d): ret=%d\r\n", DEFAULT_MPU_HZ, ret);

   ret = dmp_load_motion_driver_firmware();
   printf("[DMP] dmp_load_firmware: ret=%d\r\n", ret);

   ret = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
   printf("[DMP] dmp_set_orientation: ret=%d\r\n", ret);

   ret = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
       DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
       DMP_FEATURE_GYRO_CAL);
   printf("[DMP] dmp_enable_feature: ret=%d\r\n", ret);

   ret = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
   printf("[DMP] dmp_set_fifo_rate: ret=%d\r\n", ret);

   ret = mpu_set_dmp_state(1);
   printf("[DMP] mpu_set_dmp_state(1): ret=%d\r\n", ret);

   printf("[DMP] DMP_Init complete\r\n");
}

/**************************************************************************
Function: Read DMP attitude data (Roll, Pitch, Yaw)
         After a successful read the internal snapshot is updated atomically.
**************************************************************************/
static uint32_t dmp_read_cnt = 0;
static uint32_t dmp_ok_cnt = 0;
static uint32_t dmp_fail_cnt = 0;

void Read_DMP(float *Pitch_out, float *Roll_out, float *Yaw_out)
{
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];
	int ret;

	ret = dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
	dmp_read_cnt++;

	/* Print detail for first 10 calls, then every 200th */
	if (dmp_read_cnt <= 10 || dmp_read_cnt % 200 == 0)
	{
		printf("[RD] #%lu ret=%d sensors=0x%04X more=%d\r\n",
		       (unsigned long)dmp_read_cnt, ret, (unsigned)sensors, (int)more);
	}

	if (ret != 0)
	{
		dmp_fail_cnt++;
		if (dmp_fail_cnt <= 5)
			printf("[RD] !!! dmp_read_fifo FAILED ret=%d (fail#%lu)\r\n",
			       ret, (unsigned long)dmp_fail_cnt);
		return;
	}

	if (sensors & INV_WXYZ_QUAT)
	{
		q0 = quat[0] / q30;
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30;

		Pitch = atan2(2 * q2 * q3 + 2 * q0 * q1,
		              -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3f;
		Roll  = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3f;
		Yaw   = atan2(2 * (q1 * q2 + q0 * q3),
		              q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.3f;

		*Pitch_out = Pitch;
		*Roll_out  = Roll;
		*Yaw_out   = Yaw;

		dmp_ok_cnt++;
		if (dmp_ok_cnt <= 3)
			printf("[RD] quat OK #%lu: q=(%ld,%ld,%ld,%ld)\r\n",
			       (unsigned long)dmp_ok_cnt,
			       quat[0], quat[1], quat[2], quat[3]);

		/* ---- update snapshot with interrupt guard ---- */
		__disable_irq();
		euler_snapshot.roll  = Roll;
		euler_snapshot.pitch = Pitch;
		euler_snapshot.yaw   = Yaw;
		euler_ready = 1;
		__enable_irq();
	}
	else
	{
		if (dmp_read_cnt <= 10)
			printf("[RD] no QUAT flag, sensors=0x%04X\r\n", (unsigned)sensors);
	}
}

/**************************************************************************
Function: MPU6050_GetEuler - safe output interface for other modules
         Copies a consistent {roll, pitch, yaw} into *out.
Returns : 1 = valid data, 0 = DMP not ready yet
**************************************************************************/
uint8_t MPU6050_GetEuler(MPU6050_Euler_t *out)
{
	if (!euler_ready)
		return 0;

	__disable_irq();
	*out = euler_snapshot;
	__enable_irq();
	return 1;
}

/**************************************************************************
Function: Read MPU6050 temperature sensor
**************************************************************************/
int Read_Temperature(void)
{
	  float Temp;
	  Temp=(I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_L);
		if(Temp>32768) Temp-=65536;
		Temp=(36.53+Temp/340)*10;
	  return (int)Temp;
}

/**************************************************************************
Function: MPU6050_Init
**************************************************************************/
void MPU6050_Init(void)
{
	uint8_t id;
	printf("[MPU] MPU6050_Init start\r\n");
	id = MPU6050_getDeviceID();
	printf("[MPU] WHO_AM_I = 0x%02X (expect 0x68)\r\n", id);
	MPU6050_initialize();
	printf("[MPU] MPU6050_Init done\r\n");
}
