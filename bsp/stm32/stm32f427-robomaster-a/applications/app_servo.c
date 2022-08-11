#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define LOG_TAG              "servo"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>


#define SERVO1_4_PWM        "pwm4"
#define SERVO2_8_PWM        "pwm5"
     

struct rt_device_pwm *pwm_servo1_4;
struct rt_device_pwm *pwm_servo2_8;

rt_thread_t app_servo_thread;

void app_servo_entry(void* param);
int app_servo_init(void)
{
	int8_t ret=0;
 	pwm_servo1_4 = (struct rt_device_pwm *)rt_device_find(SERVO1_4_PWM);
	pwm_servo2_8 = (struct rt_device_pwm *)rt_device_find(SERVO2_8_PWM);
	
	if(pwm_servo1_4 == RT_NULL || pwm_servo2_8 == RT_NULL)
	{
		LOG_E("servo not found");
        return -1;
	}	
    app_servo_thread  = rt_thread_create("servo", app_servo_entry , RT_NULL, 512, 1, 10);

    if(app_servo_thread == RT_NULL)
    {
        LOG_E("buzzer thread create faild");
        return -1;
    }
    rt_thread_startup(app_servo_thread);
	return 0;
}
//INIT_APP_EXPORT(app_servo_init);

void app_servo_entry(void* param)
{
    int32_t count = 0;
	int32_t sta =50000;

	rt_pwm_enable(pwm_servo1_4, 1);
	rt_pwm_enable(pwm_servo1_4, 2);
	rt_pwm_enable(pwm_servo1_4, 3);
	rt_pwm_enable(pwm_servo1_4, 4);
	
	rt_pwm_enable(pwm_servo2_8, 1);
	rt_pwm_enable(pwm_servo2_8, 2);
	rt_pwm_enable(pwm_servo2_8, 3);
	rt_pwm_enable(pwm_servo2_8, 4);
    while(1)
    {
		
		for(uint32_t i =5;i<25;i++)
		{
			rt_pwm_set(pwm_servo1_4, 1, 20000000, i*100000);
			rt_pwm_set(pwm_servo1_4, 2, 20000000, i*100000);
			rt_pwm_set(pwm_servo1_4, 3, 20000000, i*100000);
			rt_pwm_set(pwm_servo1_4, 4, 20000000, i*100000);
			rt_pwm_set(pwm_servo2_8, 1, 20000000, i*100000);
			rt_pwm_set(pwm_servo2_8, 2, 20000000, i*100000);
			rt_pwm_set(pwm_servo2_8, 3, 20000000, i*100000);
			rt_pwm_set(pwm_servo2_8, 4, 20000000, i*100000);
	
			rt_thread_mdelay(500);
		}
    }
}


