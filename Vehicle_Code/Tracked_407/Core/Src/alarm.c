//
// alarm.c
// 报警 + YOLO 串口接收实现
//

#include "alarm.h"

/* ================================================================== */
/*                     全局标志                                        */
/* ================================================================== */
volatile uint8_t g_yolo_object_found_flag = 0;

/* ================================================================== */
/*                    YOLO 接收私有变量                                 */
/* ================================================================== */
static uint8_t  s_yolo_rx_byte = 0;
static uint8_t  s_yolo_frame[YOLO_FRAME_LEN];
static uint8_t  s_yolo_index = 0;

/* ================================================================== */
/*                        报警函数实现                                  */
/* ================================================================== */
void System_Init_Alarm(void)
{
    for (int i = 0; i < SYSTEM_INIT_ALARM_TIMES; i++) {
        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
        HAL_Delay(200);
    }
}

void Object_Found_Alarm(void)
{
    for (int i = 0; i < OBJECT_FOUND_ALAEM_TIMES; i++) {
        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
        HAL_Delay(100);
    }
}

/* ================================================================== */
/*                    YOLO 帧解析（私有）                               */
/* ================================================================== */
/**
 * @brief YOLO 逐字节解析
 * @note  协议: [0xAA] [cmd] [param] [checksum]
 *        checksum = byte0 ^ byte1 ^ byte2
 */
static void YOLO_ParseByte(uint8_t byte)
{
    /* 帧头对齐 */
    if (s_yolo_index == 0U) {
        if (byte != YOLO_FRAME_HEAD) {
            return;
        }
    }

    s_yolo_frame[s_yolo_index++] = byte;

    if (s_yolo_index >= YOLO_FRAME_LEN) {
        s_yolo_index = 0U;

        /* 校验和 */
        uint8_t sum = s_yolo_frame[0] ^ s_yolo_frame[1] ^ s_yolo_frame[2];
        if (sum != s_yolo_frame[3]) {
            return;
        }

        /* 只处理"检测到物体"命令 */
        if (s_yolo_frame[1] == YOLO_CMD_OBJECT_FOUND) {
            g_yolo_object_found_flag = 1;    /* 只置标志，不报警 */
        }
    }
}

/* ================================================================== */
/*                    YOLO 公共接口实现                                 */
/* ================================================================== */
void YOLO_UART_Start(void)
{
    s_yolo_index = 0;
    HAL_UART_Receive_IT(YOLO_UART, &s_yolo_rx_byte, 1);
}

void YOLO_RxHandler(void)
{
    /* 解析当前字节 */
    YOLO_ParseByte(s_yolo_rx_byte);
    /* 重新开启接收 */
    HAL_UART_Receive_IT(YOLO_UART, &s_yolo_rx_byte, 1);
}