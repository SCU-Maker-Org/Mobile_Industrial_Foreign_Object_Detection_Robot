//
// Created by s on 2026/9/23.
//

#ifndef TRACKED_IMU406_H
#define TRACKED_IMU406_H

#include <stdint.h>
#include <stdbool.h>
#include "usart.h"

typedef int32_t IMU406;     /* 类型别名 */

/* ================================================================== */
/*                   ★ 配置区：在这里选择使用哪个 UART ★                */
/* ================================================================== */
/* 可选值：2 或 3，对应 USART2 / USART3                                */
#define IMU406_UART_NUM     2

/* 根据 IMU406_UART_NUM 自动选择 huart 和初始化函数 */
#if   (IMU406_UART_NUM == 2)
    #define IMU406_HUART            (&huart2)
    #define IMU406_UART_INIT()      MX_USART2_UART_Init()
#elif (IMU406_UART_NUM == 3)
    #define IMU406_HUART            (&huart3)
    #define IMU406_UART_INIT()      MX_USART3_UART_Init()
#else
    #error "IMU406_UART_NUM 只能是 2 或 3"
#endif

/* ------------------------------------------------------------------ */
/*                     协议配置                                        */
/* ------------------------------------------------------------------ */
#define IMU406_FRAME_LEN     29U
#define IMU406_FRAME_HEAD    0x84

/* ------------------------------------------------------------------ */
/*                     数据全局变量（定义在 .c 中）                     */
/* ------------------------------------------------------------------ */
extern volatile IMU406 IMU406_ROLL;
extern volatile IMU406 IMU406_PITCH;
extern volatile IMU406 IMU406_YAW;
extern volatile IMU406 IMU406_ACC_X;
extern volatile IMU406 IMU406_ACC_Y;
extern volatile IMU406 IMU406_ACC_Z;
extern volatile IMU406 IMU406_GYRO_X;
extern volatile IMU406 IMU406_GYRO_Y;
extern volatile IMU406 IMU406_GYRO_Z;

/* ------------------------------------------------------------------ */
/*                     公共函数声明                                     */
/* ------------------------------------------------------------------ */
void IMU406_Init(void);          /* 初始化 UART 并开启接收中断 */
void IMU406_Rx_ISR(void);        /* 在 UART 中断里调用 */

/* 单数据获取函数 */
IMU406 IMU406_Get_Roll(void);
IMU406 IMU406_Get_Pitch(void);
IMU406 IMU406_Get_Yaw(void);
IMU406 IMU406_Get_AccX(void);
IMU406 IMU406_Get_AccY(void);
IMU406 IMU406_Get_AccZ(void);
IMU406 IMU406_Get_GyroX(void);
IMU406 IMU406_Get_GyroY(void);
IMU406 IMU406_Get_GyroZ(void);

#endif // TRACKED_IMU406_H