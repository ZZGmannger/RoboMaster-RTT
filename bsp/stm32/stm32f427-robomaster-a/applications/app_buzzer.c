#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define LOG_TAG              "buzzer"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>


#define BUZZER_PWM        "pwm12"
#define BUZZER_PWM_CH     1       
struct rt_device_pwm *pwm_buzzer;
rt_thread_t app_buzzer_thread;



void app_buzzer_entry(void* param);
int app_buzzer_init(void)
{
	int8_t ret=0;
 	pwm_buzzer = (struct rt_device_pwm *)rt_device_find(BUZZER_PWM);
	
	if(pwm_buzzer == RT_NULL)
	{
		LOG_E("buzzer not found");
        return -1;
	}	
    app_buzzer_thread  = rt_thread_create("motor", app_buzzer_entry , RT_NULL, 512, 1, 10);

    if(app_buzzer_thread == RT_NULL)
    {
        LOG_E("buzzer thread create faild");
        return -1;
    }
    rt_thread_startup(app_buzzer_thread);
	return 0;
}
//INIT_APP_EXPORT(app_buzzer_init);

void app_buzzer_entry(void* param)
{
    int32_t count = 0;
	int32_t sta =50000;

	rt_pwm_enable(pwm_buzzer, BUZZER_PWM_CH);
    while(1)
    {
		rt_pwm_set(pwm_buzzer, BUZZER_PWM_CH, 500000, 0);
        rt_thread_mdelay(500);
		rt_pwm_set(pwm_buzzer, BUZZER_PWM_CH, 500000, 400000);
        rt_thread_mdelay(500);
    }
}


