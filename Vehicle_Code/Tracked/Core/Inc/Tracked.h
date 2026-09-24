//
// Created by s on 2026/9/23.
//

#ifndef TRACKED_TRACKED_H
#define TRACKED_TRACKED_H

#include <stdint.h>
#include "PID.h"

/* ================================================================== */
/*                     机械参数（根据实际小车修改）                     */
/* ================================================================== */
#define TRACKED_WHEEL_BASE      0.23f   /* 左右轮距，单位 m */
#define TRACKED_WHEEL_DIAMETER  0.065f  /* 轮径，单位 m */
#define TRACKED_ENCODER_PPR     13.0f   /* 编码器每圈脉冲数（单相） */
#define TRACKED_GEAR_RATIO      30.0f   /* 减速比 */
#define TRACKED_DT              0.05f   /* 控制周期 20ms */

/* ================================================================== */
/*                     任务周期（ms）                                   */
/* ================================================================== */
#define TRACKED_PID_PERIOD_MS   50U     /* PID 周期 */
#define TRACKED_IMU_PERIOD_MS   10U     /* IMU 周期 */
#define TRACKED_IMU_OFFSET_MS   5U      /* IMU 与 PID 错开 5ms */
#define TRACKED_ROS_PERIOD_MS   50U     /* ROS 发送周期 */

/* ================================================================== */
/*                     安全：指令超时                                   */
/* ================================================================== */
#define TRACKED_CMD_TIMEOUT_MS  60000U

/* ================================================================== */
/*                     ROS 上行：里程计+IMU 帧（50ms 发一次）           */
/* ================================================================== */
typedef struct __attribute__((packed)) {
    uint8_t  header[2];      /* 0xAA 0x55 帧头 */
    float    x;              /* 里程计 x (m) */
    float    y;              /* 里程计 y (m) */
    float    yaw;            /* 里程计偏航角 (rad) */
    float    v;              /* 线速度 (m/s) */
    float    w;              /* 角速度 (rad/s) */
    float    roll;           /* IMU 翻滚角 */
    float    pitch;          /* IMU 俯仰角 */
    float    imu_yaw;        /* IMU 偏航角 */
    int32_t  left_cnt;       /* 左轮累计计数 */
    int32_t  right_cnt;      /* 右轮累计计数 */
    uint8_t  checksum;       /* 校验和 */
} Tracked_ROS_Frame_t;

/* ================================================================== */
/*                     ROS 下行：速度指令帧（ROS → STM32）              */
/* ================================================================== */
#define TRACKED_CMD_HEAD0       0xAAU
#define TRACKED_CMD_HEAD1       0x55U
#define TRACKED_CMD_TYPE_VEL    0x01U
#define TRACKED_CMD_TAIL        0x0DU
#define TRACKED_CMD_FRAME_LEN   13U
/* 布局：
 *   [0]    0xAA
 *   [1]    0x55
 *   [2]    0x01  命令类型：速度
 *   [3..6] float v  (小端)
 *   [7..10] float w (小端)
 *   [11]   checksum = buf[0..10] 异或
 *   [12]   0x0D  帧尾
 */

/* 接收状态机 */
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
    /* ---- 机械参数 ---- */
    float wheel_base;
    float wheel_diameter;
    float encoder_ppr;
    float gear_ratio;
    float dt;

    /* ---- 目标速度 ---- */
    float target_v;
    float target_w;

    /* ---- 当前速度 ---- */
    float current_v;
    float current_w;

    /* ---- 左右轮目标/实际线速度 ---- */
    float target_vl, target_vr;
    float current_vl, current_vr;

    /* ---- 编码器（增量值，每次读取后归零） ---- */
    int32_t delta_left_cnt;      /* 本次采样周期内的左轮增量 */
    int32_t delta_right_cnt;     /* 本次采样周期内的右轮增量 */
    int32_t total_left_cnt;      /* 累计总计数（用于里程计/显示） */
    int32_t total_right_cnt;     /* 累计总计数（用于里程计/显示） */

    /* ---- PID ---- */
    PID_Speed_Controller pid_left;
    PID_Speed_Controller pid_right;

    /* ---- IMU ---- */
    float roll, pitch, yaw;

    /* ---- 里程计 ---- */
    float odom_x, odom_y, odom_yaw;

    /* ---- 运行状态 ---- */
    uint8_t enabled;

    /* ---- ROS 接收状态机 ---- */
    Tracked_RxState_t rx_state;
    uint8_t  rx_buf[TRACKED_CMD_FRAME_LEN];
    uint8_t  rx_index;
    uint32_t last_cmd_tick;      /* 最后一次收到有效指令的时间 */
    uint8_t  cmd_timeout;        /* 1 = 已超时，已停车 */
    uint8_t  cmd_received_flag;  /* 1 = 本周期收到过指令（调试用） */
} Tracked_t;

/* ================================================================== */
/*                     全局小车对象                                     */
/* ================================================================== */
extern Tracked_t g_tracked;

/* ================================================================== */
/*                     公共接口                                         */
/* ================================================================== */

/* 初始化 */
void Tracked_Init(void);

/* 主循环调用（非阻塞，内部按时间片调度）—— 用 SysTick 方案时可不调 */
void Tracked_Loop(void);

/* ★ SysTick 调度入口：在 SysTick_Handler 里调用（1ms 一次） */
void Tracked_Tick(void);

/* 设置目标速度（由 ROS 接收或上层调用） */
void Tracked_SetTargetSpeed(float v, float w);

/* 正运动学：左右轮线速度 → 车体 v/w */
void Tracked_ForwardKinematics(float vl, float vr, float *v, float *w);

/* 逆运动学：车体 v/w → 左右轮线速度 */
void Tracked_InverseKinematics(float v, float w, float *vl, float *vr);

/* 读取编码器增量（读后立即归零），并直接计算轮速 */
void Tracked_UpdateEncoder(void);

/* PID 更新（内部调用） */
void Tracked_PID_Update(void);

/* 读取 IMU（内部调用） */
void Tracked_UpdateIMU(void);

/* 更新里程计 */
void Tracked_UpdateOdom(void);

/* 向 ROS 发送一帧（内部调用） */
void Tracked_SendToROS(void);

/* 使能/失能 */
void Tracked_Enable(uint8_t en);

/* ================================================================== */
/*                     ROS 下行接收接口                                 */
/* ================================================================== */

/* 启动 USART1 接收中断（在 Tracked_Init 里自动调用） */
void Tracked_StartRosRx(void);

/* 在 USART1 中断里喂一个字节（由 HAL_UART_RxCpltCallback 调用） */
void Tracked_FeedByte(uint8_t byte);

/* 获取目标速度（调试/监控用） */
float Tracked_GetTargetV(void);
float Tracked_GetTargetW(void);

/* ================================================================== */
/*                     USART1 接收缓冲（供中断文件使用）                */
/* ================================================================== */
extern uint8_t g_ros_rx_byte;

#endif //TRACKED_TRACKED_H