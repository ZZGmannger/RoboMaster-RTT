#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define LOG_TAG              "power"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>

#define POWER24_1_PIN    GET_PIN(H,2)
#define POWER24_2_PIN    GET_PIN(H,3)
#define POWER24_3_PIN    GET_PIN(H,4)
#define POWER24_4_PIN    GET_PIN(H,5)

rt_thread_t app_power_thread;
void app_power_entry(void* param);
int app_power_init(void)
{
	int8_t ret=0;

    rt_pin_mode(POWER24_1_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(POWER24_2_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(POWER24_3_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(POWER24_4_PIN, PIN_MODE_OUTPUT);

    app_power_thread  = rt_thread_create("motor", app_power_entry , RT_NULL, 512, 1, 10);

    if(app_power_thread == RT_NULL)
    {
        LOG_E("power thread create faild");
        return -1;
    }
    rt_thread_startup(app_power_thread);
	return 0;
}
//INIT_APP_EXPORT(app_power_init);

void app_power_entry(void* param)
{
	int pa2 =GET_PIN(A,2);
	int pa3 =GET_PIN(A,3);
	int pi7 =GET_PIN(I,7);
	int pi2 =GET_PIN(I,2);
	LOG_W("pa2:%d , pa3:%d , pi7:%d , pi2:%d" ,pa2,pa3,pi7,pi2);
    while(1)
    {
//        rt_pin_write(POWER24_1_PIN, PIN_HIGH);
//        rt_pin_write(POWER24_2_PIN, PIN_HIGH);
//        rt_pin_write(POWER24_3_PIN, PIN_HIGH);
//        rt_pin_write(POWER24_4_PIN, PIN_HIGH);
//        rt_thread_mdelay(1000);
//        rt_pin_write(POWER24_1_PIN, PIN_LOW);
//        rt_pin_write(POWER24_2_PIN, PIN_LOW);
//        rt_pin_write(POWER24_3_PIN, PIN_LOW);
//        rt_pin_write(POWER24_4_PIN, PIN_LOW);
        rt_thread_mdelay(1000);
    }
}


