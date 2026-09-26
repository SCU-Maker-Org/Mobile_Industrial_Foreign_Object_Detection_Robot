//
// Tracked.h
//

#ifndef TRACKED_TRACKED_H
#define TRACKED_TRACKED_H

#include <stdint.h>
#include "main.h"
#include "PID.h"

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
#define TRACKED_IMU_PERIOD_MS   10U
#define TRACKED_IMU_OFFSET_MS   5U
#define TRACKED_ROS_PERIOD_MS   50U
#define TRACKED_CMD_TIMEOUT_MS  60000U

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
typedef struct __attribute__((packed)) {
    uint8_t  header[2];
    float    x;
    float    y;
    float    yaw;
    float    v;
    float    w;
    float    roll;
    float    pitch;
    float    imu_yaw;
    int32_t  left_cnt;
    int32_t  right_cnt;
    uint8_t  checksum;
} Tracked_ROS_Frame_t;

/* ================================================================== */
/*                     ROS 下行帧                                       */
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

    float target_v, target_w;
    float current_v, current_w;
    float target_vl, target_vr;
    float current_vl, current_vr;

    int32_t delta_left_cnt;
    int32_t delta_right_cnt;
    int32_t total_left_cnt;
    int32_t total_right_cnt;

    PID_Speed_Controller pid_left;
    PID_Speed_Controller pid_right;

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
void Tracked_SendToROS(void);
void Tracked_Enable(uint8_t en);

void Tracked_StartRosRx(void);
void Tracked_FeedByte(uint8_t byte);

extern uint8_t g_ros_rx_byte;

#endif //TRACKED_TRACKED_H