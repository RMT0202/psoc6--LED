/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-06-29     Rbb666       first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#include "drv_gpio.h"

#define LED_PIN     GET_PIN(0, 1)  // LED控制引脚
#define LED_PIN1     GET_PIN(0, 0)
#define IR_PIN      GET_PIN(10, 0)  // 红外模块信号引脚(P0.2)
#define DETECT_DISTANCE_CM 5       // 检测距离阈值

static volatile rt_bool_t obstacle_near = RT_FALSE;

static void irq_handler(void *args)
{
    // 硬件消抖处理(20ms)
    static rt_tick_t last_tick = 0;
    rt_tick_t now = rt_tick_get();
    if (now - last_tick < RT_TICK_PER_SECOND/50) return;
    last_tick = now;

    // 低电平表示检测到障碍物
    obstacle_near = (rt_pin_read(IR_PIN) == PIN_LOW);
}

int main(void)
{
    // 初始化硬件
    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED_PIN1, PIN_MODE_OUTPUT);
    rt_pin_mode(IR_PIN, PIN_MODE_INPUT_PULLUP);

    // 配置红外中断(双边沿触发)
    rt_pin_attach_irq(IR_PIN, PIN_IRQ_MODE_RISING_FALLING,
                     irq_handler, RT_NULL);
    rt_pin_irq_enable(IR_PIN, PIN_IRQ_ENABLE);

    while (1) {
        // 根据障碍物状态控制LED
        rt_pin_write(LED_PIN, obstacle_near ? PIN_HIGH : PIN_LOW);
        rt_thread_mdelay(5);
        rt_pin_write(LED_PIN1, obstacle_near ? PIN_HIGH : PIN_LOW);
        rt_thread_mdelay(5);
    }
}
