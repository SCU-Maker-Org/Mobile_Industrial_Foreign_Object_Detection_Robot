//
// Created by s on 2026/9/23.
//

#include "Tracked.h"
#include "tim.h"
#include "usart.h"
#include "IMU406.h"
#include "main.h"
#include <string.h>
#include <math.h>

/* ================================================================== */
/*                     全局小车对象                                     */
/* ================================================================== */
Tracked_t g_tracked;

/* ================================================================== */
/*                     私有变量                                         */
/* ================================================================== */
static uint32_t s_last_pid_tick = 0;
static uint32_t s_last_imu_tick = 0;
static uint32_t s_last_ros_tick = 0;

/* USART1 单字节接收缓冲（供中断使用） */
static uint8_t s_ros_rx_byte = 0;

/* ================================================================== */
/*                     私有函数声明                                     */
/* ================================================================== */
static void Tracked_SetPWM(int16_t left, int16_t right);
static int32_t Tracked_ReadEncoderLeft(void);
static int32_t Tracked_ReadEncoderRight(void);
static void Tracked_SendBytes(const uint8_t *buf, uint16_t len);
static uint8_t Tracked_Checksum(const uint8_t *buf, uint16_t len);
static void Tracked_ParseCmd(const uint8_t *buf);

/* ================================================================== */
/*                     初始化                                           */
/* ================================================================== */
void Tracked_Init(void)
{
    Tracked_t *t = &g_tracked;
    memset(t, 0, sizeof(Tracked_t));

    /* 机械参数 */
    t->wheel_base     = TRACKED_WHEEL_BASE;
    t->wheel_diameter = TRACKED_WHEEL_DIAMETER;
    t->encoder_ppr    = TRACKED_ENCODER_PPR;
    t->gear_ratio     = TRACKED_GEAR_RATIO;
    t->dt             = TRACKED_DT;

    /* PID 参数（需实际调） */
    PID_Speed_Controller_Init(&t->pid_left,  10.0f, 0.5f, 0.0f, 100.0f, 1000.0f);
    PID_Speed_Controller_Init(&t->pid_right, 10.0f, 0.5f, 0.0f, 100.0f, 1000.0f);

    /* 启动 PWM */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    /* 启动编码器 */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    /* IMU 初始化（会开启 USART2 接收中断） */
    IMU406_Init();

    /* ---- ROS 接收状态机初始化 ---- */
    t->rx_state     = TRACKED_RX_WAIT_HEAD0;
    t->rx_index     = 0;
    t->last_cmd_tick = HAL_GetTick();
    t->cmd_timeout   = 0;
    t->cmd_received_flag = 0;

    /* ---- 启动 USART1 接收中断（ROS 下行） ---- */
    Tracked_StartRosRx();

    /* 初始化时间戳 */
    s_last_pid_tick = HAL_GetTick();
    s_last_imu_tick = HAL_GetTick();
    s_last_ros_tick = HAL_GetTick();

    t->enabled = 1;
}

/* ================================================================== */
/*                     主循环调度                                       */
/* ================================================================== */
void Tracked_Loop(void)
{
    uint32_t now = HAL_GetTick();

    /* ---- 指令超时检查 ---- */
    if ((now - g_tracked.last_cmd_tick) > TRACKED_CMD_TIMEOUT_MS) {
        if (!g_tracked.cmd_timeout) {
            g_tracked.cmd_timeout = 1;
            Tracked_SetTargetSpeed(0.0f, 0.0f);   /* 超时停车 */
        }
    }

    /* ---- 10ms PID ---- */
    if ((now - s_last_pid_tick) >= TRACKED_PID_PERIOD_MS) {
        s_last_pid_tick += TRACKED_PID_PERIOD_MS;
        Tracked_UpdateEncoder();
        Tracked_PID_Update();
        Tracked_UpdateOdom();
    }

    /* ---- 10ms IMU，偏移 5ms ---- */
    if ((now - s_last_imu_tick) >= TRACKED_IMU_PERIOD_MS) {
        if (((now + TRACKED_IMU_OFFSET_MS) % TRACKED_IMU_PERIOD_MS) == 0U) {
            s_last_imu_tick += TRACKED_IMU_PERIOD_MS;
            Tracked_UpdateIMU();
        }
    }

    /* ---- 50ms ROS ---- */
    if ((now - s_last_ros_tick) >= TRACKED_ROS_PERIOD_MS) {
        s_last_ros_tick += TRACKED_ROS_PERIOD_MS;
        Tracked_SendToROS();
    }
}

/* ================================================================== */
/*                     正运动学                                         */
/* ================================================================== */
void Tracked_ForwardKinematics(float vl, float vr, float *v, float *w)
{
    *v = (vl + vr) * 0.5f;
    *w = (vr - vl) / g_tracked.wheel_base;
}

/* ================================================================== */
/*                     逆运动学                                         */
/* ================================================================== */
void Tracked_InverseKinematics(float v, float w, float *vl, float *vr)
{
    *vl = v - (w * g_tracked.wheel_base * 0.5f);
    *vr = v + (w * g_tracked.wheel_base * 0.5f);
}

/* ================================================================== */
/*                     设置目标速度                                     */
/* ================================================================== */
void Tracked_SetTargetSpeed(float v, float w)
{
    g_tracked.target_v = v;
    g_tracked.target_w = w;
    Tracked_InverseKinematics(v, w, &g_tracked.target_vl, &g_tracked.target_vr);
}

/* ================================================================== */
/*                     读取编码器并计算轮速                             */
/* ================================================================== */
void Tracked_UpdateEncoder(void)
{
    Tracked_t *t = &g_tracked;

    int32_t l = Tracked_ReadEncoderLeft();
    int32_t r = Tracked_ReadEncoderRight();

    int32_t dl = l - t->last_left_cnt;
    int32_t dr = r - t->last_right_cnt;

    t->last_left_cnt  = l;
    t->last_right_cnt = r;

    t->total_left_cnt  += dl;
    t->total_right_cnt += dr;

    /* 每圈脉冲数 = PPR * 减速比 * 4（四倍频） */
    float ppr_total = t->encoder_ppr * t->gear_ratio * 4.0f;
    float wheel_circumference = 3.1415926f * t->wheel_diameter;

    /* 脉冲 → 轮线速度 (m/s) */
    t->current_vl = ((float)dl / ppr_total) * wheel_circumference / t->dt;
    t->current_vr = ((float)dr / ppr_total) * wheel_circumference / t->dt;

    /* 正运动学 */
    Tracked_ForwardKinematics(t->current_vl, t->current_vr,
                              &t->current_v, &t->current_w);
}

/* ================================================================== */
/*                     PID 更新                                         */
/* ================================================================== */
void Tracked_PID_Update(void)
{
    Tracked_t *t = &g_tracked;

    if (!t->enabled) {
        Tracked_SetPWM(0, 0);
        return;
    }

    float out_l = PID_Speed_Controller_Update(&t->pid_left,
                                              t->target_vl, t->current_vl);
    float out_r = PID_Speed_Controller_Update(&t->pid_right,
                                              t->target_vr, t->current_vr);

    Tracked_SetPWM((int16_t)out_l, (int16_t)out_r);
}

/* ================================================================== */
/*                     读取 IMU                                         */
/* ================================================================== */
void Tracked_UpdateIMU(void)
{
    Tracked_t *t = &g_tracked;

    /* IMU406 数据由中断解析到全局变量，这里直接读 */
    t->roll  = (float)IMU406_Get_Roll()  / 100.0f;
    t->pitch = (float)IMU406_Get_Pitch() / 100.0f;
    t->yaw   = (float)IMU406_Get_Yaw()   / 100.0f;
}

/* ================================================================== */
/*                     里程计更新                                       */
/* ================================================================== */
void Tracked_UpdateOdom(void)
{
    Tracked_t *t = &g_tracked;

    float v = t->current_v;
    float w = t->current_w;

    /* 中点积分 */
    t->odom_x   += v * cosf(t->odom_yaw) * t->dt;
    t->odom_y   += v * sinf(t->odom_yaw) * t->dt;
    t->odom_yaw += w * t->dt;

    /* 归一化到 [-pi, pi] */
    while (t->odom_yaw >  3.1415926f) t->odom_yaw -= 2.0f * 3.1415926f;
    while (t->odom_yaw < -3.1415926f) t->odom_yaw += 2.0f * 3.1415926f;
}

/* ================================================================== */
/*                     向 ROS 发送结构体                                */
/* ================================================================== */
void Tracked_SendToROS(void)
{
    Tracked_t *t = &g_tracked;

    Tracked_ROS_Frame_t frame;
    frame.header[0] = 0xAA;
    frame.header[1] = 0x55;
    frame.x         = t->odom_x;
    frame.y         = t->odom_y;
    frame.yaw       = t->odom_yaw;
    frame.v         = t->current_v;
    frame.w         = t->current_w;
    frame.roll      = t->roll;
    frame.pitch     = t->pitch;
    frame.imu_yaw   = t->yaw;
    frame.left_cnt  = t->total_left_cnt;
    frame.right_cnt = t->total_right_cnt;
    frame.checksum  = Tracked_Checksum((uint8_t *)&frame,
                                       sizeof(frame) - 1);

    Tracked_SendBytes((uint8_t *)&frame, sizeof(frame));
}

/* ================================================================== */
/*                     使能/失能                                        */
/* ================================================================== */
void Tracked_Enable(uint8_t en)
{
    g_tracked.enabled = en;
    if (!en) {
        Tracked_SetPWM(0, 0);
        PID_Speed_Controller_Init(&g_tracked.pid_left,  10.0f, 0.5f, 0.0f, 100.0f, 1000.0f);
        PID_Speed_Controller_Init(&g_tracked.pid_right, 10.0f, 0.5f, 0.0f, 100.0f, 1000.0f);
    }
}

/* ================================================================== */
/*                     ROS 下行接收接口                                 */
/* ================================================================== */

/* 启动 USART1 接收中断 */
void Tracked_StartRosRx(void)
{
    HAL_UART_Receive_IT(&huart1, &s_ros_rx_byte, 1);
}

/* 获取 USART1 接收缓冲地址（供中断文件使用） */
uint8_t *Tracked_GetRosRxBuf(void)
{
    return &s_ros_rx_byte;
}

/* 逐字节接收状态机 */
void Tracked_FeedByte(uint8_t byte)
{
    Tracked_t *t = &g_tracked;
    uint8_t *buf = t->rx_buf;

    switch (t->rx_state) {

    case TRACKED_RX_WAIT_HEAD0:
        if (byte == TRACKED_CMD_HEAD0) {
            buf[0] = byte;
            t->rx_index = 1;
            t->rx_state = TRACKED_RX_WAIT_HEAD1;
        }
        break;

    case TRACKED_RX_WAIT_HEAD1:
        if (byte == TRACKED_CMD_HEAD1) {
            buf[1] = byte;
            t->rx_index = 2;
            t->rx_state = TRACKED_RX_WAIT_TYPE;
        } else {
            t->rx_state = TRACKED_RX_WAIT_HEAD0;
        }
        break;

    case TRACKED_RX_WAIT_TYPE:
        if (byte == TRACKED_CMD_TYPE_VEL) {
            buf[2] = byte;
            t->rx_index = 3;
            t->rx_state = TRACKED_RX_WAIT_DATA;
        } else {
            t->rx_state = TRACKED_RX_WAIT_HEAD0;
        }
        break;

    case TRACKED_RX_WAIT_DATA:
        buf[t->rx_index++] = byte;
        /* 数据区 = v(4)+w(4)+cks(1) = 9 字节
           从 index=3 收到 index=11，共 9 字节 */
        if (t->rx_index >= 12U) {
            t->rx_state = TRACKED_RX_WAIT_TAIL;
        }
        break;

    case TRACKED_RX_WAIT_TAIL:
        if (byte == TRACKED_CMD_TAIL) {
            buf[12] = byte;
            Tracked_ParseCmd(buf);
        }
        t->rx_state = TRACKED_RX_WAIT_HEAD0;
        t->rx_index = 0;
        break;

    default:
        t->rx_state = TRACKED_RX_WAIT_HEAD0;
        t->rx_index = 0;
        break;
    }
}

/* 获取目标速度（调试用） */
float Tracked_GetTargetV(void) { return g_tracked.target_v; }
float Tracked_GetTargetW(void) { return g_tracked.target_w; }

/* ================================================================== */
/*                     私有函数实现                                     */
/* ================================================================== */

/* 解析速度指令帧 */
static void Tracked_ParseCmd(const uint8_t *buf)
{
    /* 校验和：buf[0..10] 异或 == buf[11] */
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 11U; i++) {
        sum ^= buf[i];
    }
    if (sum != buf[11]) {
        return;                 /* 校验失败，丢弃 */
    }

    float v, w;
    memcpy(&v, &buf[3], 4);
    memcpy(&w, &buf[7], 4);

    Tracked_SetTargetSpeed(v, w);

    g_tracked.last_cmd_tick = HAL_GetTick();
    g_tracked.cmd_timeout   = 0;
    g_tracked.cmd_received_flag = 1;
}

/* 设置 PWM 并控制方向引脚 */
static void Tracked_SetPWM(int16_t left, int16_t right)
{
    uint16_t pwm_l = (left  < 0) ? (uint16_t)(-left)  : (uint16_t)left;
    uint16_t pwm_r = (right < 0) ? (uint16_t)(-right) : (uint16_t)right;

    if (pwm_l > 65535) pwm_l = 65535;
    if (pwm_r > 65535) pwm_r = 65535;

    /* 左轮方向 */
    if (left >= 0) {
        HAL_GPIO_WritePin(LEFT_1_GPIO_Port, LEFT_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LEFT_2_GPIO_Port, LEFT_2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(LEFT_1_GPIO_Port, LEFT_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LEFT_2_GPIO_Port, LEFT_2_Pin, GPIO_PIN_SET);
    }

    /* 右轮方向 */
    if (right >= 0) {
        HAL_GPIO_WritePin(RIGHT_1_GPIO_Port, RIGHT_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RIGHT_2_GPIO_Port, RIGHT_2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(RIGHT_1_GPIO_Port, RIGHT_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RIGHT_2_GPIO_Port, RIGHT_2_Pin, GPIO_PIN_SET);
    }

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_l);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm_r);
}

/* 读左编码器（TIM2） */
static int32_t Tracked_ReadEncoderLeft(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
}

/* 读右编码器（TIM3） */
static int32_t Tracked_ReadEncoderRight(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
}

/* 通过 USART1 发送字节 */
static void Tracked_SendBytes(const uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100);
}

/* 校验和 */
static uint8_t Tracked_Checksum(const uint8_t *buf, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum ^= buf[i];
    }
    return sum;
}