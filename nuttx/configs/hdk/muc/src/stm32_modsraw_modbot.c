/*
 * Copyright (c) 2017 Motorola Mobility, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <errno.h>
#include <debug.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>

#include <unistd.h>

#include <arch/board/mods.h>

#include <nuttx/arch.h>
#include <nuttx/pwm.h>
#include <nuttx/device.h>
#include <nuttx/device_raw.h>
#include <nuttx/power/pm.h>

#include "stm32_pwm.h"
#include "stm32_tim.h"
#include <sys/ioctl.h>

#define PWM_L        0
#define PWM_R        1
#define PWM_DEVICES  2

/*#define GPIO_MODBOT_AIN1         CALC_GPIO_NUM('B', 10)
#define GPIO_MODBOT_AIN2         CALC_GPIO_NUM('B', 11)
#define GPIO_MODBOT_BIN1         CALC_GPIO_NUM('A', 4)
#define GPIO_MODBOT_BIN2         CALC_GPIO_NUM('A', 5)*/

#define PWM_TIM_LEFT  6
#define PWM_TIM_RIGHT 7

#define MOTOR_STOP 0
#define MOTOR_FORWARD 1
#define MOTOR_BACKWARD 2


#define BLINKY_ACTIVITY    10

static int8_t cur_left = MOTOR_STOP;
static int8_t cur_right = MOTOR_STOP;

static bool modbot_started = false;

//static int motor_config[][] = {{GPIO_MODBOT_AIN1, GPIO_MODBOT_AIN2} , {GPIO_MODBOT_BIN1, GPIO_MODBOT_BIN2}};
static int motor_config_forward[] = {GPIO_MODBOT_AIN1, GPIO_MODBOT_BIN1};
static int motor_config_backward[] = {GPIO_MODBOT_AIN2, GPIO_MODBOT_BIN2};
static int motor_timer_num[] = {PWM_TIM_LEFT, PWM_TIM_RIGHT};
static int motor_period[] = {1000, 1000};
static int motor_freq[] = {1000, 1000};

static struct stm32_tim_dev_s *tim_dev_left;
static struct stm32_tim_dev_s *tim_dev_right;

static int motor_left_timer_handler(int irq, FAR void *context)
{
    uint8_t new_val;

    pm_activity(BLINKY_ACTIVITY);
    STM32_TIM_ACKINT(tim_dev_left, 0);

    new_val = gpio_get_value(motor_config_forward[0]) ^ 1;
    gpio_set_value(motor_config_forward[0], new_val);

    //llvdbg("new_val=%d\n", new_val);
    return 0;
}

static int motor_right_timer_handler(int irq, FAR void *context)
{
    uint8_t new_val;

    pm_activity(BLINKY_ACTIVITY);
    STM32_TIM_ACKINT(tim_dev_right, 0);

    new_val = gpio_get_value(motor_config_forward[1]) ^ 1;
    gpio_set_value(motor_config_forward[1], new_val);

    //llvdbg("new_val=%d\n", new_val);
    return 0;
}

// modbot_stop will stop the PWM signal generation and place the motor driver
// in standby.  This should be called to disable the motors
static void modbot_stop(void)
{
    modbot_started = false;

    gpio_set_value(GPIO_MODS_DEMO_ENABLE, 0);

    if (tim_dev_left) {
        dbg("STOP\n");

        STM32_TIM_DISABLEINT(tim_dev_left, 0);
        stm32_tim_deinit(tim_dev_left);
        tim_dev_left = NULL;
    } else {
        dbg("ignore\n");
    }

    if (tim_dev_right) {
        dbg("STOP\n");

        STM32_TIM_DISABLEINT(tim_dev_right, 0);
        stm32_tim_deinit(tim_dev_right);
        tim_dev_right = NULL;
    } else {
        dbg("ignore\n");
    }

    gpio_set_value(GPIO_MODBOT_STANDBY, 0);
    gpio_set_value(motor_config_forward[0], 0);
    gpio_set_value(motor_config_forward[1], 0);
    gpio_set_value(motor_config_backward[0], 0);
    gpio_set_value(motor_config_backward[1], 0);
}

// modbot_start will enable PWM signal generation and bring the 
// motor controller out of Standby in preparation to drive
static void modbot_start(void)
{
    bool success = true;

    gpio_set_value(GPIO_MODS_DEMO_ENABLE, 1);
    

    if (!tim_dev_left) {
        dbg("LEFT\n");

        tim_dev_left = stm32_tim_init(motor_timer_num[0]);

        DEBUGASSERT(tim_dev_left);

        STM32_TIM_SETPERIOD(tim_dev_left, motor_period[0]);
        STM32_TIM_SETCLOCK(tim_dev_left, motor_freq[0]);
        STM32_TIM_SETMODE(tim_dev_left, STM32_TIM_MODE_PULSE);
        STM32_TIM_SETISR(tim_dev_left, motor_left_timer_handler, 0);
        STM32_TIM_ENABLEINT(tim_dev_left, 0);
    } else {
        dbg("ignore\n");
    }

    if (!tim_dev_right) {
        dbg("RIGHT\n");

        tim_dev_right = stm32_tim_init(motor_timer_num[1]);

        DEBUGASSERT(tim_dev_right);

        STM32_TIM_SETPERIOD(tim_dev_right, motor_period[1]);
        STM32_TIM_SETCLOCK(tim_dev_right, motor_freq[1]);
        STM32_TIM_SETMODE(tim_dev_right, STM32_TIM_MODE_PULSE);
        STM32_TIM_SETISR(tim_dev_right, motor_right_timer_handler, 0);
        STM32_TIM_ENABLEINT(tim_dev_right, 0);
    } else {
        dbg("ignore\n");
    }

    modbot_started = success;
    if (success) {
        gpio_set_value(GPIO_MODBOT_STANDBY, 1);
        gpio_set_value(motor_config_forward[0], 0);
        gpio_set_value(motor_config_forward[1], 0);
        gpio_set_value(motor_config_backward[0], 0);
        gpio_set_value(motor_config_backward[1], 0);
    } else {
        // Some error occurred, reset all to stopped state
        modbot_stop();
    }
}

static int modbot_recv(struct device *dev, uint32_t len, uint8_t data[])
{
    if(modbot_started == false){
        modbot_start();
    }
    int i;
    int8_t sdata[2] = {0,0};
    int val;
    unsigned int uval;

    if (len < 2)
        return -EINVAL;

    if (modbot_started == false) {
        return 0;
    }

    for (i = 0; i < PWM_DEVICES; i++) {
        sdata[i] = (int8_t)data[i];
        val = abs(sdata[i]);
        uval = (unsigned int)val;
        if (uval > 100) {
            uval = 100;
        }
        if (uval != 0) {
            //pwm_info[i].duty = uitoub16(uval)/100;
            //ioctl(pwm_fd[i], PWMIOC_SETCHARACTERISTICS,
            //        (unsigned long)&pwm_info[i]);
        }
    }

    if (sdata[PWM_L] != cur_left) {
        cur_left = sdata[PWM_L];
        if (cur_left > 0) {                     // Forward
            vdbg("Left Forward: %d\n", cur_left);
            // gpio_set_value(GPIO_MODBOT_AIN1, 0);  /* Motors wired backwards! */
            // gpio_set_value(GPIO_MODBOT_AIN2, 1);
        } else if (cur_left < 0) {           // Backward
            vdbg("Left Backward: %d\n", cur_left);
            // gpio_set_value(GPIO_MODBOT_AIN2, 0);
            // gpio_set_value(GPIO_MODBOT_AIN1, 1);
        } else {                                // Stop
            vdbg("Left Stop: %d\n", cur_left);
            // gpio_set_value(GPIO_MODBOT_AIN1, 0);  /* 0/0 will coast */  
            // gpio_set_value(GPIO_MODBOT_AIN2, 0);  /* 1/1 will brake hard */

        }
    }

    if (sdata[PWM_R] != cur_right) {
        cur_right = sdata[PWM_R];
        if (cur_right > 0) {                  // Forward
            vdbg("Right Forward: %d\n", cur_right);
            // gpio_set_value(GPIO_MODBOT_BIN1, 0);  /* Motors wired backwards! */
            // gpio_set_value(GPIO_MODBOT_BIN2, 1);
        } else if (cur_right < 0) {           // Backward
            vdbg("Right Backward: %d\n", cur_right);
            // gpio_set_value(GPIO_MODBOT_BIN2, 0);
            // gpio_set_value(GPIO_MODBOT_BIN1, 1);
        } else {                                // Stop
            vdbg("Right Stop: %d\n", cur_right);
            // gpio_set_value(GPIO_MODBOT_BIN1, 0);  /* 0/0 will coast */  
            // gpio_set_value(GPIO_MODBOT_BIN2, 0);  /* 1/1 will brake hard */
        }
    }

    return 0;
}

static int modbot_register_callback(struct device *dev,
                                    raw_send_callback callback)
{
    /* Nothing to do */
    return 0;
}

static int modbot_unregister_callback(struct device *dev)
{
    /* Nothing to do */
    return 0;
}

static int modbot_probe(struct device *dev)
{
    // int ret;
    // int i;

    dbg("PROBE\n");

    gpio_direction_out(GPIO_MODS_DEMO_ENABLE, 0);
    gpio_direction_out(GPIO_MODBOT_STANDBY, 0);
    gpio_direction_out(GPIO_MODBOT_AIN1, 0);
    gpio_direction_out(GPIO_MODBOT_AIN2, 0);
    gpio_direction_out(GPIO_MODBOT_BIN1, 0);
    gpio_direction_out(GPIO_MODBOT_BIN2, 0);
    gpio_set_value(motor_config_forward[0], 0);
    gpio_set_value(motor_config_forward[1], 0);
    gpio_set_value(motor_config_backward[0], 0);
    gpio_set_value(motor_config_backward[1], 0);

    // for (i = 0; i < PWM_DEVICES; i++) {
    //     if (!pwm[i]) {
    //         dbg("Creating pwm %d\n",i);
    //         pwm[i] = stm32_pwminitialize(pwm_tim[i]);
    //         if (!pwm[i]) {
    //             dbg("Failed to get the STM32 PWM %d lower half\n",i);
    //             return -ENODEV;
    //         }
    //         ret = pwm_register(pwm_devices[i], pwm[i]);
    //         if (ret < 0) {
    //             adbg("pwm_register %d failed: %d\n",i, ret);
    //             pwm[i] = NULL;
    //             return ret;
    //         }
    //     }
    // }

    // User exercise.  Delay calling modbot_start() until the Mod is
    // attached to the Moto Z.  See mods_attach_register().
    

    return 0;
}

static void modbot_remove(struct device *dev) {
    modbot_stop();
}

static struct device_raw_type_ops modbot_type_ops = {
    .recv = modbot_recv,
    .register_callback = modbot_register_callback,
    .unregister_callback = modbot_unregister_callback,
};

static struct device_driver_ops modbot_driver_ops = {
    .probe = modbot_probe,
    .remove = modbot_remove,
    .type_ops = &modbot_type_ops,
};

struct device_driver mods_raw_modbot_driver = {
    .type = DEVICE_TYPE_RAW_HW,
    .name = "mods_raw_modbot",
    .desc = "Modbot Raw Interface",
    .ops = &modbot_driver_ops,
};
