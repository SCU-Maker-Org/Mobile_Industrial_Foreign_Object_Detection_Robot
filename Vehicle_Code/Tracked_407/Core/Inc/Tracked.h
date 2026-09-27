//
// Tracked.h
//

#ifndef TRACKED_TRACKED_H
#define TRACKED_TRACKED_H

#include <stdint.h>
#include "main.h"
#include "PID.h"
#include "tim.h"
#include "usart.h"
#include "IMU406.h"
#include <string.h>
#include <math.h>
#include "alarm.h"

/* ================================================================== */
/*                     机械参数                                         */
/* ================================================================== */
#define TRACKED_WHEEL_BASE      0.23f
#define TRACKED_WHEEL_DIAMETER  0.065f
#define TRACKED_ENCODER_PPR     13.0f
#define TRACKED_GEAR_RATIO      30.0f
#define TRACKED_DT              0.05f

/* ================================================================== */
/*                     任务周期                                         */
/* ================================================================== */
#define TRACKED_PID_PERIOD_MS   50U
#define TRACKED_IMU_PERIOD_MS   10U        /* ★ IMU 以 100Hz 发送 */
#define TRACKED_ROS_PERIOD_MS   50U
#define TRACKED_CMD_TIMEOUT_MS  60000U

/* ================================================================== */
/*                     ★ PID 参数（集中调参）                          */
/* ================================================================== */

/* ---- 内环：轮子级线速度 PID ---- */
#define PID_WHEEL_KP            500.0f
#define PID_WHEEL_KI            50.0f
#define PID_WHEEL_KD            2.0f
#define PID_WHEEL_INTEGRAL_LIM  999.0f
#define PID_WHEEL_OUTPUT_LIM    999.0f

/* ---- 外环：车体线速度 PID ---- */
#define PID_BODY_V_KP           0.0f      /* ★ 调参阶段先设 0 */
#define PID_BODY_V_KI           0.0f
#define PID_BODY_V_KD           0.0f
#define PID_BODY_V_INTEGRAL_LIM 50.0f
#define PID_BODY_V_OUTPUT_LIM   0.3f      /* 修正量最大 ±0.3 m/s */

/* ---- 外环：车体角速度 PID ---- */
#define PID_BODY_W_KP           0.0f      /* ★ 调参阶段先设 0 */
#define PID_BODY_W_KI           0.0f
#define PID_BODY_W_KD           0.0f
#define PID_BODY_W_INTEGRAL_LIM 50.0f
#define PID_BODY_W_OUTPUT_LIM   0.3f      /* 修正量最大 ±0.3 rad/s */

/* ================================================================== */
/*                     电机 / 编码器 / 串口 宏                          */
/* ================================================================== */
#define MOTOR_A_TIM_IN1         (&htim11)
#define MOTOR_A_CH_IN1          TIM_CHANNEL_1
#define MOTOR_A_TIM_IN2         (&htim10)
#define MOTOR_A_CH_IN2          TIM_CHANNEL_1

#define MOTOR_B_TIM             (&htim9)
#define MOTOR_B_CH_IN1          TIM_CHANNEL_1
#define MOTOR_B_CH_IN2          TIM_CHANNEL_2

#define MOTOR_A_ENCODER_TIM     (&htim2)
#define MOTOR_B_ENCODER_TIM     (&htim3)

#define TRACKED_ROS_UART        (&huart3)

/* ================================================================== */
/*                     ROS 上行帧                                       */
/* ================================================================== */
#define TRACKED_UP_HEAD0        0xAAU
#define TRACKED_UP_HEAD1        0x55U
#define TRACKED_UP_TYPE_ODOM    0x01U
#define TRACKED_UP_TYPE_IMU     0x02U

/* ---- 里程计上行帧 ---- */
typedef struct __attribute__((packed)) {
    uint8_t  header[2];      /* 0xAA 0x55 */
    uint8_t  type;           /* 0x01 */
    float    x;              /* 累计 x (m) */
    float    y;              /* 累计 y (m) */
    float    yaw;            /* 累计 yaw (rad) */
    float    v;              /* 线速度 (m/s) */
    float    w;              /* 角速度 (rad/s) */
    uint32_t timestamp_ms;
    uint8_t  checksum;
} Tracked_Odom_Frame_t;

/* ---- IMU 上行帧 ---- */
typedef struct __attribute__((packed)) {
    uint8_t  header[2];      /* 0xAA 0x55 */
    uint8_t  type;           /* 0x02 */
    float    gyro_x;         /* rad/s */
    float    gyro_y;         /* rad/s */
    float    gyro_z;         /* rad/s */
    float    acc_x;          /* m/s² */
    float    acc_y;          /* m/s² */
    float    acc_z;          /* m/s² */
    float    yaw;            /* rad，IMU 自带角度（融合时可参考） */
    uint32_t timestamp_ms;
    uint8_t  checksum;
} Tracked_Imu_Frame_t;

/* ================================================================== */
/*                     ROS 下行帧（/cmd_vel）                           */
/* ================================================================== */
#define TRACKED_CMD_HEAD0       0xAAU
#define TRACKED_CMD_HEAD1       0x55U
#define TRACKED_CMD_TYPE_VEL    0x01U
#define TRACKED_CMD_TAIL        0x0DU
#define TRACKED_CMD_FRAME_LEN   13U

typedef enum {
    TRACKED_RX_WAIT_HEAD0 = 0,
    TRACKED_RX_WAIT_HEAD1,
    TRACKED_RX_WAIT_TYPE,
    TRACKED_RX_WAIT_DATA,
    TRACKED_RX_WAIT_TAIL,
} Tracked_RxState_t;

/* ================================================================== */
/*                     小车结构体                                       */
/* ================================================================== */
typedef struct {
    float wheel_base;
    float wheel_diameter;
    float encoder_ppr;
    float gear_ratio;
    float dt;

    float target_v, target_w;           /* 车体级目标 */
    float current_v, current_w;         /* 车体级实际 */
    float target_vl, target_vr;         /* 轮子级目标（保留，调试用） */
    float current_vl, current_vr;       /* 轮子级实际 */

    int32_t delta_left_cnt;
    int32_t delta_right_cnt;
    int32_t total_left_cnt;
    int32_t total_right_cnt;

    PID_Speed_Controller pid_left;      /* 左轮线速度 PID */
    PID_Speed_Controller pid_right;     /* 右轮线速度 PID */

    /* ★ 车体级 PID */
    PID_Speed_Controller pid_body_v;    /* 车体线速度 PID */
    PID_Speed_Controller pid_body_w;    /* 车体角速度 PID */

    float odom_x, odom_y, odom_yaw;

    uint8_t enabled;

    Tracked_RxState_t rx_state;
    uint8_t  rx_buf[TRACKED_CMD_FRAME_LEN];
    uint8_t  rx_index;
    uint32_t last_cmd_tick;
    uint8_t  cmd_timeout;
    uint8_t  cmd_received_flag;
} Tracked_t;

extern Tracked_t g_tracked;

/* ================================================================== */
/*                     公共接口                                         */
/* ================================================================== */
void Tracked_Init(void);
void Tracked_Loop(void);
void Tracked_Tick(void);
void Tracked_SetTargetSpeed(float v, float w);
void Tracked_ForwardKinematics(float vl, float vr, float *v, float *w);
void Tracked_InverseKinematics(float v, float w, float *vl, float *vr);
void Tracked_UpdateEncoder(void);
void Tracked_PID_Update(void);
void Tracked_UpdateOdom(void);

void Tracked_SendOdom(void);
void Tracked_SendImu(void);

void Tracked_Enable(uint8_t en);

void Tracked_StartRosRx(void);
void Tracked_FeedByte(uint8_t byte);

extern uint8_t g_ros_rx_byte;

#endif // TRACKED_TRACKED_H