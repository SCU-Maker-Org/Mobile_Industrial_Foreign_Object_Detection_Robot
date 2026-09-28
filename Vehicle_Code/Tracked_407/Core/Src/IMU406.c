//
// Created by s on 2026/9/23.
//

#include "IMU406.h"

/* ------------------------------------------------------------------ */
/*                   接收状态机（私有变量）                             */
/* ------------------------------------------------------------------ */
static uint8_t g_rxByte  = 0;
static uint8_t s_rxBuf[IMU406_FRAME_LEN];
static uint8_t s_rxIndex = 0;

/* ------------------------------------------------------------------ */
/*                ★ Yaw 软件零点偏移（私有）                           */
/* ------------------------------------------------------------------ */
static volatile IMU406 s_yaw_offset = 0;

/* ------------------------------------------------------------------ */
/*                   私有函数声明                                       */
/* ------------------------------------------------------------------ */
static void   IMU406_ParseFrame(const uint8_t *frame);
static IMU406 DecodeBytes(const uint8_t *p);

/* ------------------------------------------------------------------ */
/*                   全局数据定义                                       */
/* ------------------------------------------------------------------ */
volatile IMU406 IMU406_ROLL   = 0;
volatile IMU406 IMU406_PITCH  = 0;
volatile IMU406 IMU406_YAW    = 0;
volatile IMU406 IMU406_ACC_X  = 0;
volatile IMU406 IMU406_ACC_Y  = 0;
volatile IMU406 IMU406_ACC_Z  = 0;
volatile IMU406 IMU406_GYRO_X = 0;
volatile IMU406 IMU406_GYRO_Y = 0;
volatile IMU406 IMU406_GYRO_Z = 0;

/* ------------------------------------------------------------------ */
/*                   公共函数实现                                       */
/* ------------------------------------------------------------------ */
void IMU406_Init(void)
{
    IMU406_UART_INIT();
    HAL_UART_Receive_IT(IMU406_HUART, &g_rxByte, 1);
}

void IMU406_Rx_ISR(void)
{
    if (s_rxIndex == 0U) {
        if (g_rxByte != IMU406_FRAME_HEAD) {
            goto _next;
        }
    }

    s_rxBuf[s_rxIndex++] = g_rxByte;

    if (s_rxIndex >= IMU406_FRAME_LEN) {
        IMU406_ParseFrame(s_rxBuf);
        s_rxIndex = 0U;
    }

_next:
    HAL_UART_Receive_IT(IMU406_HUART, &g_rxByte, 1);
}

/* ------------------------------------------------------------------ */
/*                   私有函数实现                                       */
/* ------------------------------------------------------------------ */
static void IMU406_ParseFrame(const uint8_t *frame)
{
    const uint8_t *p = &frame[1];

    IMU406_ROLL  = DecodeBytes(p);      p += 3;
    IMU406_PITCH = DecodeBytes(p);      p += 3;
    IMU406_YAW   = DecodeBytes(p);      p += 3;

    IMU406_ACC_X = DecodeBytes(p);      p += 3;
    IMU406_ACC_Y = DecodeBytes(p);      p += 3;
    IMU406_ACC_Z = DecodeBytes(p);      p += 3;

    IMU406_GYRO_X = DecodeBytes(p);     p += 3;
    IMU406_GYRO_Y = DecodeBytes(p);     p += 3;
    IMU406_GYRO_Z = DecodeBytes(p);
}

static IMU406 DecodeBytes(const uint8_t *p)
{
    IMU406 neg = (p[0] & 0xF0) ? 1U : 0U;

    IMU406 val =
            (p[0] & 0x0F) * 10000U +
            (p[1] >> 4)   * 1000U  +
            (p[1] & 0x0F) * 100U   +
            (p[2] >> 4)   * 10U    +
            (p[2] & 0x0F);

    return neg ? -(IMU406) val : (IMU406) val;
}

/* ------------------------------------------------------------------ */
/*                   单数据获取（原始 LSB 值）                          */
/* ------------------------------------------------------------------ */
IMU406 IMU406_Get_Roll(void)  { return IMU406_ROLL;   }
IMU406 IMU406_Get_Pitch(void) { return IMU406_PITCH;  }

/* ★ 校正后的 yaw（减掉零点偏移） */
IMU406 IMU406_Get_Yaw(void)
{
    IMU406 raw = IMU406_YAW - s_yaw_offset;
    /* LSB = 0.01° → 180° = 18000, 360° = 36000 */
    while (raw >  18000) raw -= 36000;
    while (raw < -18000) raw += 36000;
    return raw;
}

IMU406 IMU406_Get_AccX(void)  { return IMU406_ACC_X;  }
IMU406 IMU406_Get_AccY(void)  { return IMU406_ACC_Y;  }
IMU406 IMU406_Get_AccZ(void)  { return IMU406_ACC_Z;  }
IMU406 IMU406_Get_GyroX(void) { return IMU406_GYRO_X; }
IMU406 IMU406_Get_GyroY(void) { return IMU406_GYRO_Y; }
IMU406 IMU406_Get_GyroZ(void) { return IMU406_GYRO_Z; }

/* ------------------------------------------------------------------ */
/*               ★ 物理量获取（ROS 标准单位）                          */
/* ------------------------------------------------------------------ */
float IMU406_Get_GyroX_RadS(void) { return IMU406_GYRO_TO_RAD_S(IMU406_Get_GyroX()); }
float IMU406_Get_GyroY_RadS(void) { return IMU406_GYRO_TO_RAD_S(IMU406_Get_GyroY()); }
float IMU406_Get_GyroZ_RadS(void) { return IMU406_GYRO_TO_RAD_S(IMU406_Get_GyroZ()); }

float IMU406_Get_AccX_m_s2(void)  { return IMU406_ACC_TO_M_S2(IMU406_Get_AccX()); }
float IMU406_Get_AccY_m_s2(void)  { return IMU406_ACC_TO_M_S2(IMU406_Get_AccY()); }
float IMU406_Get_AccZ_m_s2(void)  { return IMU406_ACC_TO_M_S2(IMU406_Get_AccZ()); }

float IMU406_Get_Yaw_Rad(void)    { return IMU406_ANGLE_TO_RAD(IMU406_Get_Yaw()); }

/* ------------------------------------------------------------------ */
/*               ★ Yaw 零点控制                                        */
/* ------------------------------------------------------------------ */
/**
 * @brief 把当前 yaw 设为零点
 * @note  可以在任意时刻调用（建议在 ROS /tracked/reset_odom 触发时调用）
 */
void IMU406_ZeroYaw(void)
{
    s_yaw_offset = IMU406_YAW;
}

/**
 * @brief 清除零点偏移，恢复原始输出
 */
void IMU406_ResetYawOffset(void)
{
    s_yaw_offset = 0;
}

/**
 * @brief 原始 yaw（未经零点校正），调试用
 */
IMU406 IMU406_Get_YawRaw(void)
{
    return IMU406_YAW;
}