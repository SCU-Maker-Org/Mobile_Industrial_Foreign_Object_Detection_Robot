//
// Tracked.c
//

#include "Tracked.h"


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

    /* ---- 内环：轮子级 PID ---- */
    PID_Speed_Controller_Init(&t->pid_left,
                              PID_WHEEL_KP, PID_WHEEL_KI, PID_WHEEL_KD,
                              PID_WHEEL_INTEGRAL_LIM, PID_WHEEL_OUTPUT_LIM);
    PID_Speed_Controller_Init(&t->pid_right,
                              PID_WHEEL_KP, PID_WHEEL_KI, PID_WHEEL_KD,
                              PID_WHEEL_INTEGRAL_LIM, PID_WHEEL_OUTPUT_LIM);

    /* ---- 外环：车体级 PID ---- */
    PID_Speed_Controller_Init(&t->pid_body_v,
                              PID_BODY_V_KP, PID_BODY_V_KI, PID_BODY_V_KD,
                              PID_BODY_V_INTEGRAL_LIM, PID_BODY_V_OUTPUT_LIM);
    PID_Speed_Controller_Init(&t->pid_body_w,
                              PID_BODY_W_KP, PID_BODY_W_KI, PID_BODY_W_KD,
                              PID_BODY_W_INTEGRAL_LIM, PID_BODY_W_OUTPUT_LIM);

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

    t->rx_state      = TRACKED_RX_WAIT_HEAD0;
    t->rx_index      = 0;
    t->last_cmd_tick = HAL_GetTick();

    Tracked_StartRosRx();

    s_tick_cnt = 0;
    t->enabled = 1;
}

/* ================================================================== */
/*                     SysTick 调度（1kHz）                             */
/* ================================================================== */
void Tracked_Tick(void)
{
    s_tick_cnt++;

    /* 指令超时保护 */
    if ((HAL_GetTick() - g_tracked.last_cmd_tick) > TRACKED_CMD_TIMEOUT_MS) {
        if (!g_tracked.cmd_timeout) {
            g_tracked.cmd_timeout = 1;
            Tracked_SetTargetSpeed(0.0f, 0.0f);
        }
    }

    /* ---- 50ms 任务：PID + 里程计 ---- */
    if ((s_tick_cnt % TRACKED_PID_PERIOD_MS) == 0U) {
        Tracked_UpdateEncoder();
        Tracked_PID_Update();
        Tracked_UpdateOdom();
        Tracked_SendOdom();
    }

    /* ---- 10ms 任务：IMU 发送（100Hz） ---- */
    if ((s_tick_cnt % TRACKED_IMU_PERIOD_MS) == 0U) {
        Tracked_SendImu();
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
    /* 注意：target_vl/target_vr 不在这里计算，
     * 因为车体级闭环会在 Tracked_PID_Update 里重新计算 */
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

    /* ============================================================ */
    /* 第 1 层：车体级 v/w 闭环                                     */
    /* ============================================================ */

    /* 角速度反馈：用 IMU 陀螺仪 z 轴（不受轮子打滑影响） */
    float gyro_z = IMU406_Get_GyroZ_RadS();

    /* 线速度反馈：用轮式里程计（IMU 加速度太噪，不能直接积分） */
    float v_actual = t->current_v;

    /* 车体级修正量 */
    float v_corr = PID_Speed_Controller_Update(&t->pid_body_v,
                                               t->target_v, v_actual);
    float w_corr = PID_Speed_Controller_Update(&t->pid_body_w,
                                               t->target_w, gyro_z);

    /* 修正后的车体速度指令 */
    float v_cmd = t->target_v + v_corr;
    float w_cmd = t->target_w + w_corr;

    /* ============================================================ */
    /* 第 2 层：逆运动学，车体速度 → 左右轮速度                     */
    /* ============================================================ */

    float vl_target = v_cmd - (w_cmd * t->wheel_base * 0.5f);
    float vr_target = v_cmd + (w_cmd * t->wheel_base * 0.5f);

    /* 保存供调试用（OLED / 上位机读取） */
    t->target_vl = vl_target;
    t->target_vr = vr_target;

    /* ============================================================ */
    /* 第 3 层：轮子级线速度闭环                                    */
    /* ============================================================ */

    float out_l = PID_Speed_Controller_Update(&t->pid_left,  vl_target, t->current_vl);
    float out_r = PID_Speed_Controller_Update(&t->pid_right, vr_target, t->current_vr);

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
void Tracked_SendOdom(void)
{
    Tracked_t *t = &g_tracked;

    Tracked_Odom_Frame_t frame;
    frame.header[0] = TRACKED_UP_HEAD0;
    frame.header[1] = TRACKED_UP_HEAD1;
    frame.type      = TRACKED_UP_TYPE_ODOM;
    frame.x         = t->odom_x;
    frame.y         = t->odom_y;
    frame.yaw       = t->odom_yaw;
    frame.v         = t->current_v;
    frame.w         = t->current_w;
    frame.timestamp_ms = HAL_GetTick();
    frame.checksum  = Tracked_Checksum((uint8_t *)&frame, sizeof(frame) - 1);

    Tracked_SendBytes((uint8_t *)&frame, sizeof(frame));
}

void Tracked_SendImu(void)
{
    Tracked_Imu_Frame_t frame;
    frame.header[0] = TRACKED_UP_HEAD0;
    frame.header[1] = TRACKED_UP_HEAD1;
    frame.type      = TRACKED_UP_TYPE_IMU;

    frame.gyro_x = IMU406_Get_GyroX_RadS();
    frame.gyro_y = IMU406_Get_GyroY_RadS();
    frame.gyro_z = IMU406_Get_GyroZ_RadS();

    frame.acc_x  = IMU406_Get_AccX_m_s2();
    frame.acc_y  = IMU406_Get_AccY_m_s2();
    frame.acc_z  = IMU406_Get_AccZ_m_s2();

    frame.yaw    = IMU406_Get_Yaw_Rad();
    frame.timestamp_ms = HAL_GetTick();
    frame.checksum = Tracked_Checksum((uint8_t *)&frame, sizeof(frame) - 1);

    Tracked_SendBytes((uint8_t *)&frame, sizeof(frame));
}

void Tracked_Enable(uint8_t en)
{
    g_tracked.enabled = en;
    if (!en) {
        Tracked_SetPWM(0, 0);

        /* 重置所有 PID */
        PID_Speed_Controller_Init(&g_tracked.pid_left,
                                  PID_WHEEL_KP, PID_WHEEL_KI, PID_WHEEL_KD,
                                  PID_WHEEL_INTEGRAL_LIM, PID_WHEEL_OUTPUT_LIM);
        PID_Speed_Controller_Init(&g_tracked.pid_right,
                                  PID_WHEEL_KP, PID_WHEEL_KI, PID_WHEEL_KD,
                                  PID_WHEEL_INTEGRAL_LIM, PID_WHEEL_OUTPUT_LIM);

        PID_Speed_Controller_Init(&g_tracked.pid_body_v,
                                  PID_BODY_V_KP, PID_BODY_V_KI, PID_BODY_V_KD,
                                  PID_BODY_V_INTEGRAL_LIM, PID_BODY_V_OUTPUT_LIM);
        PID_Speed_Controller_Init(&g_tracked.pid_body_w,
                                  PID_BODY_W_KP, PID_BODY_W_KI, PID_BODY_W_KD,
                                  PID_BODY_W_INTEGRAL_LIM, PID_BODY_W_OUTPUT_LIM);
    }
}

/* ================================================================== */
/*                     ROS 接收（/cmd_vel）                             */
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
        __HAL_TIM_SET_COMPARE(MOTOR_A_TIM_IN1, MOTOR_A_CH_IN1, 0);
        __HAL_TIM_SET_COMPARE(MOTOR_A_TIM_IN2, MOTOR_A_CH_IN2, pwm_l);
    } else {
        __HAL_TIM_SET_COMPARE(MOTOR_A_TIM_IN1, MOTOR_A_CH_IN1, pwm_l);
        __HAL_TIM_SET_COMPARE(MOTOR_A_TIM_IN2, MOTOR_A_CH_IN2, 0);
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
    return -cnt;
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
    HAL_UART_Transmit(TRACKED_ROS_UART, (uint8_t *)buf, len, 10);
}

static uint8_t Tracked_Checksum(const uint8_t *buf, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) sum ^= buf[i];
    return sum;
}