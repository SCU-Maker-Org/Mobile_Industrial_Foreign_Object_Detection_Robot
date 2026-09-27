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
#define IMU406_UART_NUM     2

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
/*               ★ 单位换算系数（根据你的 IMU406 手册修改）★            */
/* ------------------------------------------------------------------ */
#define IMU406_GYRO_LSB_DEG_S   0.01f      /* 陀螺仪 1 LSB = 0.01 °/s */
#define IMU406_ACC_LSB_G        0.001f     /* 加速度 1 LSB = 0.001 g  */
#define IMU406_ANGLE_LSB_DEG    0.01f      /* 角度   1 LSB = 0.01 °   */

#define IMU406_DEG_TO_RAD       0.017453292519943295f
#define IMU406_G_TO_M_S2        9.80665f

#define IMU406_GYRO_TO_RAD_S(x)  ((float)(x) * IMU406_GYRO_LSB_DEG_S * IMU406_DEG_TO_RAD)
#define IMU406_ACC_TO_M_S2(x)    ((float)(x) * IMU406_ACC_LSB_G * IMU406_G_TO_M_S2)
#define IMU406_ANGLE_TO_RAD(x)   ((float)(x) * IMU406_ANGLE_LSB_DEG * IMU406_DEG_TO_RAD)

/* ------------------------------------------------------------------ */
/*                     数据全局变量                                     */
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
void IMU406_Init(void);
void IMU406_Rx_ISR(void);

/* 单数据获取（原始 LSB 值） */
IMU406 IMU406_Get_Roll(void);
IMU406 IMU406_Get_Pitch(void);
IMU406 IMU406_Get_Yaw(void);
IMU406 IMU406_Get_AccX(void);
IMU406 IMU406_Get_AccY(void);
IMU406 IMU406_Get_AccZ(void);
IMU406 IMU406_Get_GyroX(void);
IMU406 IMU406_Get_GyroY(void);
IMU406 IMU406_Get_GyroZ(void);

/* 直接取物理量（ROS 标准单位） */
float IMU406_Get_GyroX_RadS(void);
float IMU406_Get_GyroY_RadS(void);
float IMU406_Get_GyroZ_RadS(void);
float IMU406_Get_AccX_m_s2(void);
float IMU406_Get_AccY_m_s2(void);
float IMU406_Get_AccZ_m_s2(void);
float IMU406_Get_Yaw_Rad(void);

#endif // TRACKED_IMU406_H