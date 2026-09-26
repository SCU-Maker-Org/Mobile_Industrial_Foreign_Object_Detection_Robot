//
// Created by s on 2026/9/23.
//

#ifndef TRACKED_PID_H
#define TRACKED_PID_H

#include <stdint-gcc.h>

typedef struct {
    float Kp;               // 比例增益
    float Ki;               // 积分增益
    float Kd;               // 微分增益
    float error;            // 当前误差
    float last_error;       // 上次误差
    float prev_error;       // 上上次误差（用于微分）
    float differ_error;
    float sum_error;
    float output;           // 控制器输出
    float integral_limit;   // 积分限幅
    float output_limit;     // 输出限幅
} PID_Position_Controller;

void PID_Position_Controller_Init(PID_Position_Controller *pid,
                               float Kp, float Ki, float Kd,
                               float integral_limit,
                               float output_limit);

inline float Limit_Position(float cal_value, float limit_value);

float PID_Position_Controller_Update(PID_Position_Controller *pid,
                                  float target_val,
                                  float current_val);

typedef struct {
    float Kp;               // 比例增益
    float Ki;               // 积分增益
    float Kd;               // 微分增益
    float integral;         // 积分项累积
    float error;            // 当前误差
    float last_error;       // 上次误差
    float prev_error;       // 上上次误差（用于微分）
    float output;           // 控制器输出
    float integral_limit;   // 积分限幅
    float output_limit;     // 输出限幅
} PID_Speed_Controller;

void PID_Speed_Controller_Init(PID_Speed_Controller *pid,
                                  float Kp, float Ki, float Kd,
                                  float integral_limit,
                                  float output_limit);

float PID_Speed_Controller_Update(PID_Speed_Controller *pid,
                                  float target_val,
                                  float current_val);

#endif //TRACKED_PID_H
