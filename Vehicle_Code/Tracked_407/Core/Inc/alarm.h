//
// Created by s on 2026/9/27.
//

#ifndef TRACKED_407_ALARM_H
#define TRACKED_407_ALARM_H

#include "gpio.h"
#include "usart.h"
#include <stdint.h>

/* ================================================================== */
/*                        报警参数                                     */
/* ================================================================== */
#define SYSTEM_INIT_ALARM_TIMES     2
#define OBJECT_FOUND_ALAEM_TIMES    3

/* ================================================================== */
/*                    YOLO 串口通信协议（USART1）                       */
/* ================================================================== */
#define YOLO_UART                       (&huart1)
#define YOLO_FRAME_HEAD                 0xAAU
#define YOLO_FRAME_LEN                  4U

/* 命令类型：只保留"检测到物体" */
#define YOLO_CMD_OBJECT_FOUND           0x01U

/* ================================================================== */
/*                     全局标志（由中断置位）                           */
/* ================================================================== */
extern volatile uint8_t g_yolo_object_found_flag;

/* ================================================================== */
/*                        报警函数                                      */
/* ================================================================== */
/**
 * @brief 系统初始化完成提示
 * @note  上电后调用，蜂鸣器 + 蓝灯闪烁 5 次
 *        ★ 有 HAL_Delay，不能在中断里调用
 */
void System_Init_Alarm(void);

/**
 * @brief 检测到目标物体报警
 * @note  ★ 有 HAL_Delay，不能在中断里调用，只能在 main 循环里调用
 */
void Object_Found_Alarm(void);

/* ================================================================== */
/*                    YOLO 串口接收接口                                 */
/* ================================================================== */
/**
 * @brief 启动 USART1 接收中断
 * @note  在 main.c 初始化阶段调用一次
 */
void YOLO_UART_Start(void);

/**
 * @brief YOLO 接收中断处理（在 HAL_UART_RxCpltCallback 里调用）
 * @param huart 触发中断的 UART 句柄
 * @note  内部判断是否为 USART1，是则处理一帧 YOLO 数据
 *        把这个函数暴露出来，_it.c 里只需要一行调用
 */
void YOLO_RxHandler(void);

#endif //TRACKED_407_ALARM_H