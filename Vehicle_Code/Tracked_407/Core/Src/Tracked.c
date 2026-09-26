//
// Tracked.c
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

uint8_t g_ros_rx_byte = 0;

static uint32_t s_tick_cnt = 0;

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

    t->wheel_base     = TRACKED_WHEEL_BASE;
    t->wheel_diameter = TRACKED_WHEEL_DIAMETER;
    t->encoder_ppr    = TRACKED_ENCODER_PPR;
    t->gear_ratio     = TRACKED_GEAR_RATIO;
    t->dt             = TRACKED_DT;

    PID_Speed_Controller_Init(&t->pid_left,  80.0f, 20.0f, 0.0f, 200.0f, 999.0f);
    PID_Speed_Controller_Init(&t->pid_right, 80.0f, 20.0f, 0.0f, 200.0f, 999.0f);

    HAL_TIM_PWM_Start(MOTOR_A_TIM_IN1, MOTOR_A_CH_IN1);
    HAL_TIM_PWM_Start(MOTOR_A_TIM_IN2, MOTOR_A_CH_IN2);
    HAL_TIM_PWM_Start(MOTOR_B_TIM, MOTOR_B_CH_IN1);
    HAL_TIM_PWM_Start(MOTOR_B_TIM, MOTOR_B_CH_IN2);

    HAL_TIM_Encoder_Start(MOTOR_A_ENCODER_TIM, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(MOTOR_B_ENCODER_TIM, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COUNTER(MOTOR_A_ENCODER_TIM, 0);
    __HAL_TIM_SET_COUNTER(MOTOR_B_ENCODER_TIM, 0);

    /* IMU 初始化 */
    IMU406_Init();

    t->rx_state     = TRACKED_RX_WAIT_HEAD0;
    t->rx_index     = 0;
    t->last_cmd_tick = HAL_GetTick();

    Tracked_StartRosRx();

    s_tick_cnt = 0;
    t->enabled = 1;
}

/* ================================================================== */
/*                     SysTick 调度                                     */
/* ================================================================== */
void Tracked_Tick(void)
{
    s_tick_cnt++;

    if ((HAL_GetTick() - g_tracked.last_cmd_tick) > TRACKED_CMD_TIMEOUT_MS) {
        if (!g_tracked.cmd_timeout) {
            g_tracked.cmd_timeout = 1;
            Tracked_SetTargetSpeed(0.0f, 0.0f);
        }
    }

    if ((s_tick_cnt % TRACKED_PID_PERIOD_MS) == 0U) {
        Tracked_UpdateEncoder();
        Tracked_PID_Update();
        Tracked_UpdateOdom();
    }
}

void Tracked_Loop(void) { }

/* ================================================================== */
/*                     正/逆运动学                                     */
/* ================================================================== */
void Tracked_ForwardKinematics(float vl, float vr, float *v, float *w)
{
    *v = (vl + vr) * 0.5f;
    *w = (vr - vl) / g_tracked.wheel_base;
}

void Tracked_InverseKinematics(float v, float w, float *vl, float *vr)
{
    *vl = v - (w * g_tracked.wheel_base * 0.5f);
    *vr = v + (w * g_tracked.wheel_base * 0.5f);
}

void Tracked_SetTargetSpeed(float v, float w)
{
    g_tracked.target_v = v;
    g_tracked.target_w = w;
    Tracked_InverseKinematics(v, w, &g_tracked.target_vl, &g_tracked.target_vr);
}

/* ================================================================== */
/*                     编码器 / PID / 里程计                            */
/* ================================================================== */
void Tracked_UpdateEncoder(void)
{
    Tracked_t *t = &g_tracked;

    t->delta_left_cnt  = Tracked_ReadEncoderLeft();
    t->delta_right_cnt = Tracked_ReadEncoderRight();
    t->total_left_cnt  += t->delta_left_cnt;
    t->total_right_cnt += t->delta_right_cnt;

    float ppr_total = t->encoder_ppr * t->gear_ratio * 4.0f;
    float wheel_circumference = 3.1415926f * t->wheel_diameter;

    t->current_vl = ((float)t->delta_left_cnt  / ppr_total) * wheel_circumference / t->dt;
    t->current_vr = ((float)t->delta_right_cnt / ppr_total) * wheel_circumference / t->dt;

    Tracked_ForwardKinematics(t->current_vl, t->current_vr,
                              &t->current_v, &t->current_w);
}

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

void Tracked_UpdateOdom(void)
{
    Tracked_t *t = &g_tracked;

    float v = t->current_v;
    float w = t->current_w;

    t->odom_x   += v * cosf(t->odom_yaw) * t->dt;
    t->odom_y   += v * sinf(t->odom_yaw) * t->dt;
    t->odom_yaw += w * t->dt;

    while (t->odom_yaw >  3.1415926f) t->odom_yaw -= 2.0f * 3.1415926f;
    while (t->odom_yaw < -3.1415926f) t->odom_yaw += 2.0f * 3.1415926f;
}

/* ================================================================== */
/*                     向 ROS 发送                                       */
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

    /* 用 IMU406 的数据替代原来的 g_icm */
    frame.roll      = (float)IMU406_Get_Roll()  / 100.0f;
    frame.pitch     = (float)IMU406_Get_Pitch() / 100.0f;
    frame.imu_yaw   = (float)IMU406_Get_Yaw()   / 100.0f;

    frame.left_cnt  = t->total_left_cnt;
    frame.right_cnt = t->total_right_cnt;
    frame.checksum  = Tracked_Checksum((uint8_t *)&frame,
                                       sizeof(frame) - 1);

    Tracked_SendBytes((uint8_t *)&frame, sizeof(frame));
}

void Tracked_Enable(uint8_t en)
{
    g_tracked.enabled = en;
    if (!en) {
        Tracked_SetPWM(0, 0);
        PID_Speed_Controller_Init(&g_tracked.pid_left,  80.0f, 20.0f, 0.0f, 200.0f, 999.0f);
        PID_Speed_Controller_Init(&g_tracked.pid_right, 80.0f, 20.0f, 0.0f, 200.0f, 999.0f);
    }
}

/* ================================================================== */
/*                     ROS 接收                                          */
/* ================================================================== */
void Tracked_StartRosRx(void)
{
    HAL_UART_Receive_IT(TRACKED_ROS_UART, &g_ros_rx_byte, 1);
}

void Tracked_FeedByte(uint8_t byte)
{
    Tracked_t *t = &g_tracked;
    uint8_t *buf = t->rx_buf;

    switch (t->rx_state) {
    case TRACKED_RX_WAIT_HEAD0:
        if (byte == TRACKED_CMD_HEAD0) {
            buf[0] = byte; t->rx_index = 1;
            t->rx_state = TRACKED_RX_WAIT_HEAD1;
        }
        break;
    case TRACKED_RX_WAIT_HEAD1:
        if (byte == TRACKED_CMD_HEAD1) {
            buf[1] = byte; t->rx_index = 2;
            t->rx_state = TRACKED_RX_WAIT_TYPE;
        } else {
            t->rx_state = TRACKED_RX_WAIT_HEAD0;
        }
        break;
    case TRACKED_RX_WAIT_TYPE:
        if (byte == TRACKED_CMD_TYPE_VEL) {
            buf[2] = byte; t->rx_index = 3;
            t->rx_state = TRACKED_RX_WAIT_DATA;
        } else {
            t->rx_state = TRACKED_RX_WAIT_HEAD0;
        }
        break;
    case TRACKED_RX_WAIT_DATA:
        buf[t->rx_index++] = byte;
        if (t->rx_index >= 12U) t->rx_state = TRACKED_RX_WAIT_TAIL;
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

/* ================================================================== */
/*                     私有函数实现                                     */
/* ================================================================== */
static void Tracked_ParseCmd(const uint8_t *buf)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 11U; i++) sum ^= buf[i];
    if (sum != buf[11]) return;

    float v, w;
    memcpy(&v, &buf[3], 4);
    memcpy(&w, &buf[7], 4);

    Tracked_SetTargetSpeed(v, w);

    g_tracked.last_cmd_tick = HAL_GetTick();
    g_tracked.cmd_timeout   = 0;
    g_tracked.cmd_received_flag = 1;
}

static void Tracked_SetPWM(int16_t left, int16_t right)
{
    uint16_t pwm_l = (left  < 0) ? (uint16_t)(-left)  : (uint16_t)left;
    uint16_t pwm_r = (right < 0) ? (uint16_t)(-right) : (uint16_t)right;

    if (pwm_l > 999) pwm_l = 999;
    if (pwm_r > 999) pwm_r = 999;

    if (left >= 0) {
        __HAL_TIM_SET_COMPARE(MOTOR_A_TIM_IN1, MOTOR_A_CH_IN1, pwm_l);
        __HAL_TIM_SET_COMPARE(MOTOR_A_TIM_IN2, MOTOR_A_CH_IN2, 0);
    } else {
        __HAL_TIM_SET_COMPARE(MOTOR_A_TIM_IN1, MOTOR_A_CH_IN1, 0);
        __HAL_TIM_SET_COMPARE(MOTOR_A_TIM_IN2, MOTOR_A_CH_IN2, pwm_l);
    }

    if (right >= 0) {
        __HAL_TIM_SET_COMPARE(MOTOR_B_TIM, MOTOR_B_CH_IN1, pwm_r);
        __HAL_TIM_SET_COMPARE(MOTOR_B_TIM, MOTOR_B_CH_IN2, 0);
    } else {
        __HAL_TIM_SET_COMPARE(MOTOR_B_TIM, MOTOR_B_CH_IN1, 0);
        __HAL_TIM_SET_COMPARE(MOTOR_B_TIM, MOTOR_B_CH_IN2, pwm_r);
    }
}

static int32_t Tracked_ReadEncoderLeft(void)
{
    int32_t cnt = (int32_t)__HAL_TIM_GET_COUNTER(MOTOR_A_ENCODER_TIM);
    if (cnt > 32767) cnt -= 65536;
    __HAL_TIM_SET_COUNTER(MOTOR_A_ENCODER_TIM, 0);
    return cnt;
}

static int32_t Tracked_ReadEncoderRight(void)
{
    int32_t cnt = (int32_t)__HAL_TIM_GET_COUNTER(MOTOR_B_ENCODER_TIM);
    if (cnt > 32767) cnt -= 65536;
    __HAL_TIM_SET_COUNTER(MOTOR_B_ENCODER_TIM, 0);
    return cnt;
}

static void Tracked_SendBytes(const uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit_IT(TRACKED_ROS_UART, (uint8_t *)buf, len);
}

static uint8_t Tracked_Checksum(const uint8_t *buf, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) sum ^= buf[i];
    return sum;
}