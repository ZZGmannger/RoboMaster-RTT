/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

/* defined the LED_G pin: PF14 */
#define LED0_PIN    GET_PIN(F, 14)

#define BUZZER_PWM        "pwm12"
#define BUZZER_PWM_CH     1       
struct rt_device_pwm *pwm_buzzer;

int main(void)
{
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
	pwm_buzzer = (struct rt_device_pwm *)rt_device_find(BUZZER_PWM);
	rt_pwm_set(pwm_buzzer, BUZZER_PWM_CH, 500000, 0);
	rt_pwm_enable(pwm_buzzer, BUZZER_PWM_CH);
    while (1)
    {   
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
}
