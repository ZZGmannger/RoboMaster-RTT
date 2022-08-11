#include <rtthread.h>
#include <rtdevice.h>

#include <string.h>
#include <stdlib.h>

#define LOG_TAG              "imu"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>

#include <math.h>

#include "mpu6xxx.h"

typedef struct
{
	 /*from sensor*/
	struct 
	{
		int16_t ax;
		int16_t ay;
		int16_t az;

		int16_t gx;
		int16_t gy;
		int16_t gz;

		int16_t mx;
		int16_t my;
		int16_t mz;

		float temp;

		float wx; /*!< omiga, +- 2000dps => +-32768  so gx/16.384/57.3 =	rad/s */
		float wy;
		float wz;

		float vx;
		float vy;
		float vz;
	}raw;

	struct 
	{
		float q0;
		float q1;
		float q2;
		float q3;

		float exint;
		float eyint;
		float ezint;
		 
		float ax;
		float ay;
		float az;

		float gx;
		float gy;
		float gz;

		float mx;
		float my;
		float mz;
		uint32_t ts;
		uint32_t last_ts;
	}cal;
	
	struct 
	{
		float roll;
		float pitch;
		float yaw;
	}out;
}robot_imu_t;

void app_imu_entry(void* param);
static struct mpu6xxx_device *imu_dev;
static robot_imu_t ros_imu;
static rt_thread_t app_imu_thread;

int app_imu_init(void)
{
	imu_dev = mpu6xxx_init("spi50", RT_NULL);
	if (imu_dev == RT_NULL)
	{
		rt_kprintf("mpu6xxx init failed\n");
		return -1;
	}
	
	app_imu_thread = rt_thread_create("imu",app_imu_entry,RT_NULL , 1024 , 5 ,10);
	
    if (app_imu_thread != RT_NULL)
        rt_thread_startup(app_imu_thread);
}
//INIT_APP_EXPORT(app_imu_init);

static void imu_raw_data_get(robot_imu_t* imu)
{
	struct mpu6xxx_3axes accel, gyro,mag;
	float temp;
	
	mpu6xxx_get_accel(imu_dev, &accel);
	mpu6xxx_get_gyro(imu_dev, &gyro);
	mpu6xxx_get_mag(imu_dev, &mag);
	mpu6xxx_get_temp(imu_dev, &temp);
	
	imu->raw.ax = accel.x;
	imu->raw.ay = accel.y;
	imu->raw.az = accel.z;

	imu->raw.gx = gyro.x;
	imu->raw.gy = gyro.y;
	imu->raw.gz = gyro.z;

	imu->raw.mx = mag.x;
	imu->raw.my = mag.y;
	imu->raw.mz = mag.z;
	imu->raw.temp = temp;
    
	/*2000dps -> rad/s*/
	imu->raw.wx = imu->raw.gx / 16.384f / 57.3f;
	imu->raw.wy = imu->raw.gy / 16.384f / 57.3f;
	imu->raw.wz = imu->raw.gz / 16.384f / 57.3f;
}
float my_inv_sqrt(float x) 
{
	float halfx = 0.5f * x;
	float y     = x;
	long  i     = *(long*)&y;
	
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	
	return y;
}

static void quaternion_init(robot_imu_t* imu)
{
    if(imu->raw.mx < 0  &&  imu->raw.my <0)
	{
		if(fabs(imu->raw.mx / imu->raw.my) >=1)
		{
			imu->cal.q0 = -0.005;
			imu->cal.q1 = -0.199;
			imu->cal.q2 = 0.979;
			imu->cal.q3 = -0.0089;
		}
		else
		{
            imu->cal.q0 = -0.008;
			imu->cal.q1 = -0.555;
			imu->cal.q2 = 0.83;
			imu->cal.q3 = -0.002;
		}
	}
	else if(imu->raw.mx < 0  &&  imu->raw.my >0)
	{
		if (fabs(imu->raw.mx / imu->raw.my) >= 1)
		{
			imu->cal.q0 = 0.005;
			imu->cal.q1 = -0.199;
			imu->cal.q2 = -0.978;
			imu->cal.q3 = 0.012;
		}
		else
		{
			imu->cal.q0 = 0.005;
			imu->cal.q1 = -0.553;
			imu->cal.q2 = -0.83;
			imu->cal.q3 = -0.0023;
		}
	}
	else if(imu->raw.mx > 0  &&  imu->raw.my >0)
	{
		if (fabs(imu->raw.mx / imu->raw.my) >= 1)
		{
			imu->cal.q0 = 0.0012;
			imu->cal.q1 = -0.978;
			imu->cal.q2 = -0.199;
			imu->cal.q3 = -0.005;
		}
		else
		{
			imu->cal.q0 = 0.0023;
			imu->cal.q1 = -0.83;
			imu->cal.q2 = -0.553;
			imu->cal.q3 = 0.0023;
		}
	}
	else if(imu->raw.mx > 0  &&  imu->raw.my <0)
	{
		if (fabs(imu->raw.mx / imu->raw.my) >= 1)
		{
			imu->cal.q0 = 0.0025;
			imu->cal.q1 = 0.978;
			imu->cal.q2 = -0.199;
			imu->cal.q3 = 0.008;
		}
		else
		{
			imu->cal.q0 = 0.0025;
			imu->cal.q1 = 0.83;
			imu->cal.q2 = -0.56;
			imu->cal.q3 = 0.0045;
		}
	}
	imu->cal.q0 = -0.005;
	imu->cal.q1 = -0.199;
	imu->cal.q2 = 0.979;
	imu->cal.q3 = -0.0089;
}



void imu_update(robot_imu_t* imu)
{
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez, halfT;
	float tempq0, tempq1, tempq2, tempq3;

	float q0q0 = imu->cal.q0 * imu->cal.q0;
	float q0q1 = imu->cal.q0 * imu->cal.q1;
	float q0q2 = imu->cal.q0 * imu->cal.q2;
	float q0q3 = imu->cal.q0 * imu->cal.q3;
	float q1q1 = imu->cal.q1 * imu->cal.q1;
	float q1q2 = imu->cal.q1 * imu->cal.q2;
	float q1q3 = imu->cal.q1 * imu->cal.q3;
	float q2q2 = imu->cal.q2 * imu->cal.q2;
	float q2q3 = imu->cal.q2 * imu->cal.q3;
	float q3q3 = imu->cal.q3 * imu->cal.q3;

	imu_raw_data_get(imu);

	imu->cal.gx = imu->raw.wx;
	imu->cal.gy = imu->raw.wy;
	imu->cal.gz = imu->raw.wz;

	imu->cal.ax = imu->raw.ax;
	imu->cal.ay = imu->raw.ay;
	imu->cal.az = imu->raw.az;

	imu->cal.mx = imu->raw.mx;
	imu->cal.my = imu->raw.my;
	imu->cal.mz = imu->raw.mz;

	imu->cal.ts = rt_tick_get_millisecond(); // ms
	halfT = ((float)(imu->cal.ts - 	imu->cal.last_ts) / 2000.0f);
	imu->cal.last_ts = imu->cal.ts;

	/* Fast inverse square-root */
	norm = my_inv_sqrt(imu->cal.ax  * imu->cal.ax  + imu->cal.ay * imu->cal.ay + imu->cal.az * imu->cal.az);
	imu->cal.ax = imu->cal.ax * norm;
	imu->cal.ay = imu->cal.ay * norm;
	imu->cal.az = imu->cal.az * norm;

#if  1
	norm = my_inv_sqrt(imu->cal.mx * imu->cal.mx + imu->cal.my * imu->cal.my + imu->cal.mz * imu->cal.mz);
	imu->cal.mx = imu->cal.mx * norm;
	imu->cal.my = imu->cal.my * norm;
	imu->cal.mz = imu->cal.mz * norm;
#else
	imu->cal.mx = 0.0f;
	imu->cal.my = 0.0f;
	imu->cal.mz = 0.0f;
#endif
   	/* compute reference direction of flux */
	hx = 2.0f*imu->cal.mx*(0.5f - 
	- q3q3) + 2.0f*imu->cal.my*(q1q2 - q0q3) + 2.0f*imu->cal.mz*(q1q3 + q0q2);
	hy = 2.0f*imu->cal.mx*(q1q2 + q0q3) + 2.0f*imu->cal.my*(0.5f - q1q1 - q3q3) + 2.0f*imu->cal.mz*(q2q3 - q0q1);
	hz = 2.0f*imu->cal.mx*(q1q3 - q0q2) + 2.0f*imu->cal.my*(q2q3 + q0q1) + 2.0f*imu->cal.mz*(0.5f - q1q1 - q2q2);         
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz; 
	
	/* estimated direction of gravity and flux (v and w) */
	vx = 2.0f*(q1q3 - q0q2);
	vy = 2.0f*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
	wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
	wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
	
	/* 
	 * error is sum of cross product between reference direction 
	 * of fields and direction measured by sensors 
	 */
	ex = (imu->cal.ay*vz - imu->cal.az*vy) + (imu->cal.my*wz - imu->cal.mz*wy);
	ey = (imu->cal.az*vx - imu->cal.ax*vz) + (imu->cal.mz*wx - imu->cal.mx*wz);
	ez = (imu->cal.ax*vy - imu->cal.ay*vx) + (imu->cal.mx*wy - imu->cal.my*wx);

	/* PI */
	#define Kp 2.0f                                              
	#define Ki 0.01f  
	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		imu->cal.exint = imu->cal.exint + ex * Ki * halfT;
		imu->cal.eyint = imu->cal.eyint + ey * Ki * halfT;	
		imu->cal.ezint = imu->cal.ezint + ez * Ki * halfT;
		
		imu->cal.gx = imu->cal.gx + Kp*ex + imu->cal.exint;
		imu->cal.gy = imu->cal.gy + Kp*ey + imu->cal.eyint;
		imu->cal.gz = imu->cal.gz + Kp*ez + imu->cal.ezint;
	}
	
	tempq0 = imu->cal.q0 + (-imu->cal.q1*imu->cal.gx - imu->cal.q2*imu->cal.gy - imu->cal.q3*imu->cal.gz) * halfT;
	tempq1 = imu->cal.q1 + ( imu->cal.q0*imu->cal.gx + imu->cal.q2*imu->cal.gz - imu->cal.q3*imu->cal.gy) * halfT;
	tempq2 = imu->cal.q2 + ( imu->cal.q0*imu->cal.gy - imu->cal.q1*imu->cal.gz + imu->cal.q3*imu->cal.gx) * halfT;
	tempq3 = imu->cal.q3 + ( imu->cal.q0*imu->cal.gz + imu->cal.q1*imu->cal.gy - imu->cal.q2*imu->cal.gx) * halfT;  

	/* normalise quaternion */
	norm = my_inv_sqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
	imu->cal.q0 = tempq0 * norm;
	imu->cal.q1 = tempq1 * norm;
	imu->cal.q2 = tempq2 * norm;
	imu->cal.q3 = tempq3 * norm;

	/* yaw    -pi----pi */
	imu->out.yaw = -atan2(2*imu->cal.q1*imu->cal.q2 + 2*imu->cal.q0*imu->cal.q3, -2*imu->cal.q2*imu->cal.q2 - 2*imu->cal.q3*imu->cal.q3 + 1)* 57.3; 
	/* pitch  -pi/2----pi/2 */
	imu->out.pitch = -asin(-2*imu->cal.q1*imu->cal.q3 + 2*imu->cal.q0*imu->cal.q2)* 57.3;   
	/* roll   -pi----pi  */	
	imu->out.roll =  atan2(2*imu->cal.q2*imu->cal.q3 + 2*imu->cal.q0*imu->cal.q1, -2*imu->cal.q1*imu->cal.q1 - 2*imu->cal.q2*imu->cal.q2 + 1)* 57.3;
}
void app_imu_entry(void* param)
{
	quaternion_init(&ros_imu);
	while(1)
	{
		imu_update(&ros_imu);
		LOG_D("Roll: %8.3lf    Pitch: %8.3lf    Yaw: %8.3lf", ros_imu.out.roll, ros_imu.out.pitch, ros_imu.out.yaw);
		rt_thread_mdelay(10);
	}
}