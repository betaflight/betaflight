#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "common/maths.h"

#include "alt_ctrl.h"
#include "sensors/rangefinder.h"

#include "kalman_filter.h"


controller_t height_controller; 
controller_t vel_controller; 


float throttle_init = 0.164;
float height_setpoint = 0.50;
float vel_setpoint = 0.20;
float thrust_range = 0.005;
float height_error_range = 0.02;
float vel_error_range = 0.01;
float limit = 1;


void height_controller_init(controller_t * controller)
{
    memset(controller, 0, sizeof(controller_t));
    controller->pid.P = 10;
    controller->pid.I = 0.0;
    controller->pid.D = 0.0;

    controller->pid.Error1 = 0.0;
    controller->pid.Error2 = 0.0;
    controller->pid.iError = 0.0;

    controller->setpoint = 1.2;
    controller->output_min = -0.1;
    controller->output_max = 0.1;

    controller->input_error_range = height_error_range;
}

void vel_controller_init(controller_t * controller)
{
    memset(controller, 0, sizeof(controller_t));
    controller->pid.P = 5;
    controller->pid.I = 0.0;
    controller->pid.D = 0.0;

    controller->pid.Error1 = 0.0;
    controller->pid.Error2 = 0.0;
    controller->pid.iError = 0.0;

    controller->setpoint = 0;
    controller->throttle = 0;

    controller->output_min = -0.05;
    controller->output_max = 0.05;

    controller->input_error_range = vel_error_range;
}

void Controller_Init(void)
{
    height_controller_init(&height_controller);
    vel_controller_init(&vel_controller);
}

float pid_controller(float process_value, controller_t *controller) //增量式pid计算
{
    controller->pid.iError = controller->setpoint - process_value;  // 当前误差
    if(controller->pid.iError < controller->input_error_range && controller->pid.iError > -controller->input_error_range)
    {
        controller->pid.iError = 0;
    }
    controller->pid.DiError = controller->pid.iError - controller->pid.Error1;
    controller->pid.IiError += controller->pid.iError;
    controller->pid.IiError = constrainf(controller->pid.IiError, -limit, limit);

    controller->output = controller->pid.P * controller->pid.iError 
                        + controller->pid.I * controller->pid.IiError 
                        + controller->pid.D * controller->pid.DiError;

    // controller->output = controller->pid.P * (controller->pid.iError - controller->pid.Error1)
    //                      + controller->pid.I * controller->pid.iError
    //                      + controller->pid.D * (controller->pid.iError - 2 * controller->pid.Error1 + controller->pid.Error2);

    controller->pid.Error2 = controller->pid.Error1;
    controller->pid.Error1 = controller->pid.iError;

    controller->output = constrainf(controller->output, controller->output_min, controller->output_max);
    
    return controller->output;
}

void adjust_velocity(float vel_error_range, kalman_filter_t *filter)
{
    float output = pid_controller(filter->X_Hat_current->element[1], &vel_controller);
    
    //如果当前速度误差为正，则增加推力
    if(output > 0.001 || output < -0.001)
    {
        vel_controller.throttle = throttle_init + output;
    }
    // 如果当前速度误差为负，则减小推力
    else{
        vel_controller.throttle = throttle_init;
    }

}

void adjust_height(float height_error_range, kalman_filter_t *filter)
{
    float output = pid_controller(filter->X_Hat_current->element[0], &height_controller);

    vel_controller.setpoint = output;
    adjust_velocity(vel_error_range, &kalman_filter1); 
}

void Update_PID_Velocity(timeUs_t currentTimeUs) //100Hz
{
    UNUSED(currentTimeUs);
    adjust_velocity(vel_error_range, &kalman_filter1);
}
void Update_PID_Height(timeUs_t currentTimeUs) //25Hz
{
    UNUSED(currentTimeUs);
    adjust_height(height_error_range, &kalman_filter1);
}

float Get_Height_PID_Output(void)
{
    return height_controller.output;
}
float Get_Velocity_PID_Output(void)
{
    return vel_controller.output;
}

float Get_Velocity_throttle(void)
{
    return vel_controller.throttle;
}