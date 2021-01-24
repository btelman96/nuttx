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

#include <nuttx/config.h>
#include <sys/types.h>
#include <up_arch.h>

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

#define PWM_TIM_LEFT  6
#define PWM_TIM_RIGHT 7

#define MOTOR_STOP 0
#define MOTOR_FORWARD 1
#define MOTOR_BACKWARD 2


#define BLINKY_ACTIVITY    10

static int8_t cur_left = MOTOR_STOP;
static int8_t cur_right = MOTOR_STOP;

//for 50hz
#define DUTY_CYCLE_MIN 4194//1280ms roughly
#define DUTY_CYCLE_MAX 5636 //1720ms roughly

static bool modbot_started = false;
int fd_left;
int fd_right;
static int motor_config[] = {GPIO_MODBOT_SERVO_AIN, GPIO_MODBOT_SERVO_BIN};
static int motor_duty[] = {DUTY_CYCLE_MIN, DUTY_CYCLE_MIN};

static struct pwm_lowerhalf_s *pwm_dev_left;
static struct pwm_lowerhalf_s *pwm_dev_right;

static int map(int value, int low1, int high1, int low2, int high2){
    int mappedValue = low2 + (value - low1) * (high2 - low2) / (high1 - low1);
    if(mappedValue > high2)
        mappedValue = high2;
    if(mappedValue < low2)
        mappedValue = low2;
    return mappedValue;
}

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: pwm_devinit
 *
 * Description:
 *   All STM32 architectures must provide the following interface to work with
 *   examples/pwm.
 *
 ************************************************************************************/
static int pwm_enable(unsigned int pwm_id, uint8_t duty, bool reverse);
static void pwm_disable(int fd);
static int pwm_devinit(void);

// modbot_stop will stop the PWM signal generation and place the motor driver
// in standby.  This should be called to disable the motors
static void modbot_stop(void)
{
    modbot_started = false;

    gpio_set_value(GPIO_MODS_DEMO_ENABLE, 0);

    if (pwm_dev_left) {
        dbg("STOP\n");

        // STM32_TIM_DISABLEINT(tim_dev_left, 0);
        // stm32_tim_deinit(tim_dev_left);
        if(fd_left){
            pwm_disable(fd_left);
            fd_left = 0;
        }
        pwm_dev_left = NULL;
    } else {
        dbg("ignore\n");
    }

    if (pwm_dev_right) {
        dbg("STOP\n");

        // STM32_TIM_DISABLEINT(tim_dev_left, 0);
        // stm32_tim_deinit(tim_dev_left);
        if(fd_right){
            pwm_disable(fd_right);
            fd_right = 0;
        }
        pwm_dev_right = NULL;
    } else {
        dbg("ignore\n");
    }

    gpio_set_value(GPIO_MODBOT_STANDBY, 0);
    gpio_set_value(motor_config[PWM_L], 0);
    gpio_set_value(motor_config[PWM_R], 0);
}

// modbot_start will enable PWM signal generation and bring the 
// motor controller out of Standby in preparation to drive
static void modbot_start(void)
{
    bool success = true;

    gpio_set_value(GPIO_MODS_DEMO_ENABLE, 1);
    int ret = pwm_devinit(); //register both PWM channels. One for right, and one for left
    if(ret < 0){
        dbg("Failed to start timers\n");
    }

    modbot_started = success;
    if (success) {
        gpio_set_value(GPIO_MODBOT_STANDBY, 1);
        gpio_set_value(motor_config[PWM_L], 0);
        gpio_set_value(motor_config[PWM_R], 0);
    } else {
        // Some error occurred, reset all to stopped state
        modbot_stop();
    }
}

static int modbot_recv(struct device *dev, uint32_t len, uint8_t data[])
{
    dbg("modbot_recv\n");
    if(modbot_started == false){
        modbot_start();
    }
    int i;
    int8_t sdata[2] = {0,0};
    int val;
    unsigned int uval;

    if (len < 2){
        dbg("-EINVAL\n");
        return -EINVAL;
    }

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
            motor_duty[i] = uval;
        }
    }
    dbg("Value: %d\n", uval);

    if(pwm_dev_left){
        dbg("pwm_dev_left modify\n");
        if(fd_left)
            pwm_disable(fd_left);
        fd_left = pwm_enable(0, uval, false);
    }

    if(pwm_dev_right){
        dbg("pwm_dev_right modify\n");
        if(fd_right)
            pwm_disable(fd_right);
        fd_right = pwm_enable(1, uval, true);
    }

    if (sdata[PWM_L] != cur_left) {
        cur_left = sdata[PWM_L];
        if (cur_left > 0) {                     // Forward
            dbg("Left Forward: %d\n", cur_left);
        } else if (cur_left < 0) {           // Backward
            dbg("Left Backward: %d\n", cur_left);
        } else {                                // Stop
            dbg("Left Stop: %d\n", cur_left);
        }
    }

    if (sdata[PWM_R] != cur_right) {
        cur_right = sdata[PWM_R];
        if (cur_right > 0) {                     // Forward
            dbg("Right Forward: %d\n", cur_right);
        } else if (cur_right < 0) {           // Backward
            dbg("Right Backward: %d\n", cur_right);
        } else {                                // Stop
            dbg("Right Stop: %d\n", cur_right);
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
    gpio_direction_out(GPIO_MODBOT_SERVO_AIN, 0);
    gpio_direction_out(GPIO_MODBOT_SERVO_BIN, 0);
    gpio_set_value(motor_config[0], 0);
    gpio_set_value(motor_config[1], 0);

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

struct device_driver mods_raw_servobot_driver = {
    .type = DEVICE_TYPE_RAW_HW,
    .name = "mods_raw_modbot",
    .desc = "Modbot Raw Interface",
    .ops = &modbot_driver_ops,
};

int pwm_devinit(void)
{
  static bool initialized = false;
  struct pwm_lowerhalf_s *pwm0;
  struct pwm_lowerhalf_s *pwm1;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call stm32_pwminitialize() to get an instance of the PWM interface */

      pwm0 = stm32_pwminitialize(2);
      if (!pwm0)
        {
            dbg("Failed to get the STM32 PWM lower half 0\n");
            return -ENODEV;
        }

      pwm1 = stm32_pwminitialize(5);
      if (!pwm1)
        {
            dbg("Failed to get the STM32 PWM lower half 1\n");
            return -ENODEV;
        }

      /* Register the PWM driver at "/dev/pwm0" */

      ret = pwm_register("/dev/pwm0", pwm0);
      if (ret < 0)
        {
          adbg("pwm_register0 failed: %d\n", ret);
          return ret;
        }
      ret = pwm_register("/dev/pwm1", pwm1);
      if (ret < 0)
        {
          adbg("pwm_register1 failed: %d\n", ret);
          return ret;
        }

      /* Now we are initialized */

      initialized = true;
      pwm_dev_left = pwm0;
      pwm_dev_right = pwm1;
    }

  return OK;
}

/*
 * Returns a file descriptor for the PWM device that was opened and enabled,
 * or a negative value if an error occurred.
 * Disabling pwm is not necessary after a negative return.
 */
static int pwm_enable(unsigned int pwm_id, uint8_t duty, bool reverse) {
    ub16_t dutyScale;
    if(!reverse){
        dutyScale = (ub16_t) map(duty, -100, 100, DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);
    }
    else{
        dutyScale = (ub16_t) map(duty, 100, -100, DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);
    }
    struct pwm_info_s pwm_info;
    int ret;
    char *pwm_name;

    // pwm_info.duty = dutyScale; /* Approximately 50% duty cycle: subject to rounding errors */
    pwm_info.frequency = 50; /* 50hz */
    // if(duty == 100){
    //     pwm_info.duty = DUTY_CYCLE_MAX;
    // }
    // else if(duty > 75){
    //     pwm_info.duty = 5200;    
    // }
    // else if(duty > 50){
    //     pwm_info.duty = 4915;    
    // }
    // else if(duty > 25){
    //     pwm_info.duty = 4500;    
    // }
    // else{
    //     pwm_info.duty = DUTY_CYCLE_MIN;
    // }
    pwm_info.duty = dutyScale;

    switch(pwm_id) {
    case 0:
        pwm_name = "/dev/pwm0";
        break;
    case 1:
        pwm_name = "/dev/pwm1";
        break;
    default:
        printf("PWM: pwm_enable called with unknown pwm_id %u: valid values are 0 and 1\n",
               pwm_id);
        return -1;
    }    

    int fd = open(pwm_name, O_RDONLY);
    if (fd < 0) {
        printf("PWM: open of \"%s\" failed for pwm_id %u\n", pwm_name, pwm_id);
        return -1;
    }

    ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)&pwm_info);
    if (ret < 0) {
        printf("PWM: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
        close(fd);
        return ret;
    }

    ret = ioctl(fd, PWMIOC_START, 0);
    if (ret < 0) {
        printf("PWM: ioctl(PWMIOC_START) failed: %d\n", errno);
        close(fd);
        return ret;
    }
    return fd;
}

static void pwm_disable(int fd) {
    int ret;

    if (fd < 0) {
        printf("PWM: pwm_disable called with an invalid fd %d\n", fd);
    } else {
        ret = ioctl(fd, PWMIOC_STOP, 0);
        if (ret < 0) {
            printf("PWM: ioctl(PWMIOC_STOP) failed: %d\n", errno);
        }
        close(fd);
    }
}

