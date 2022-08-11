#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define LOG_TAG              "emotor"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>

#define ENCODE1_NAME   "pulse1"
#define ENCODE2_NAME   "pulse2"

#define ENCODE_PULSE_NUM    (600*4)
#define PULSE_DISTANCE_TRANSFORM   (100)

#define ONE_PULSE_TIME   (ENCODE_PULSE_NUM/100)

rt_thread_t motor_speed_query_thread;

struct rt_device *encode_dev1;
struct rt_device *encode_dev2;

void motor_speed_query_entry(void* param);


int app_motor_init(void)
{
	int8_t ret=0;
    encode_dev1 = rt_device_find(ENCODE1_NAME);
    encode_dev2 = rt_device_find(ENCODE2_NAME);
	

    if(encode_dev1  == RT_NULL || encode_dev2 == RT_NULL)
    {
        LOG_E("encoder not found");
        return -1;
    }

    if(rt_device_init(encode_dev1)!= RT_EOK || rt_device_init(encode_dev2)!= RT_EOK)
    {
        LOG_E("encoder init faild:encode");
        return -1;
    }
	if(ret)
	{
		LOG_E("encoder init faild:dac");
        return -1;
	}
    rt_device_open(encode_dev1,RT_DEVICE_OFLAG_RDONLY);
    rt_device_open(encode_dev2 ,RT_DEVICE_OFLAG_RDONLY);

    motor_speed_query_thread  = rt_thread_create("motor", motor_speed_query_entry , RT_NULL, 512, 1, 10);

    if(motor_speed_query_thread == RT_NULL)
    {
        LOG_E("motor thread create faild");
        return -1;
    }
    rt_thread_startup(motor_speed_query_thread);
	return 0;
}
//INIT_APP_EXPORT(app_motor_init);

void motor_speed_query_entry(void* param)
{
    int32_t count1 = 0;
	int32_t count2 =0;
    while(1)
    {
        rt_device_read(encode_dev1, 0, &count1, 1);
        rt_device_read(encode_dev2, 0, &count2, 1);
		count1 = -count1;
		count2 = -count2;

        LOG_I("motor_left  cnt is : %d" , count1);
		LOG_I("motor_right cnt is : %d" , count2);

        rt_thread_mdelay(100);
    }
}


