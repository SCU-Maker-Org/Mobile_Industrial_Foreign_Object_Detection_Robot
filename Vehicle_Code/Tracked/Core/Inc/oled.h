//
// Created by s on 2026/9/23.
//

#ifndef TRACKED_OLED_H
#define TRACKED_OLED_H

#include "font.h"
#include "string.h"
#include <stdint-gcc.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////
#define   OLED_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()

#define   GPIOx_OLED_PORT               GPIOB
#define   OLED_SCK_PIN                  OLED_SCL
#define   OLED_SCK_ON()                 HAL_GPIO_WritePin(GPIOx_OLED_PORT, OLED_SCK_PIN, GPIO_PIN_SET)
#define   OLED_SCK_OFF()                HAL_GPIO_WritePin(GPIOx_OLED_PORT, OLED_SCK_PIN, GPIO_PIN_RESET)
#define   OLED_SCK_TOGGLE()             HAL_GPIO_TogglePin(GPIOx_OLED_PORT, OLED_SCK_PIN)
#define   OLED_SDA_PIN                  OLED_SDA
#define   OLED_SDA_ON()                 HAL_GPIO_WritePin(GPIOx_OLED_PORT, OLED_SDA_PIN, GPIO_PIN_SET)
#define   OLED_SDA_OFF()                HAL_GPIO_WritePin(GPIOx_OLED_PORT, OLED_SDA_PIN, GPIO_PIN_RESET)
#define   OLED_SDA_TOGGLE()             HAL_GPIO_TogglePin(GPIOx_OLED_PORT, OLED_SDA_PIN)
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// 颜色模式定义
typedef enum {
    OLED_COLOR_NORMAL = 0, // 正常模式 黑底白字
    OLED_COLOR_REVERSED    // 反色模式 白底黑字
} OLED_ColorMode;

// 显存大小定义
#define OLED_PAGE 8            // OLED页数
#define OLED_ROW 8 * OLED_PAGE // OLED行数
#define OLED_COLUMN 128        // OLED列数

extern char oled_buffer[32];

// 基础函数
void OLED_Init();
void OLED_DisPlay_On();
void OLED_DisPlay_Off();

// 显存操作函数
void OLED_NewFrame();
void OLED_ShowFrame();
void OLED_SetPixel(uint8_t x, uint8_t y, OLED_ColorMode color);

// 图形绘制函数
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, OLED_ColorMode color);
void OLED_DrawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode color);
void OLED_DrawFilledRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode color);
void OLED_DrawTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, OLED_ColorMode color);
void OLED_DrawFilledTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, OLED_ColorMode color);
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r, OLED_ColorMode color);
void OLED_DrawFilledCircle(uint8_t x, uint8_t y, uint8_t r, OLED_ColorMode color);
void OLED_DrawEllipse(uint8_t x, uint8_t y, uint8_t a, uint8_t b, OLED_ColorMode color);
void OLED_DrawImage(uint8_t x, uint8_t y, const Image *img, OLED_ColorMode color);

// 文字绘制函数（支持颜色模式）
void OLED_PrintASCIIChar(uint8_t x, uint8_t y, char ch, const ASCIIFont *font, OLED_ColorMode color);
void OLED_PrintASCIIString(uint8_t x, uint8_t y, char *str, const ASCIIFont *font, OLED_ColorMode color);
void OLED_PrintString(uint8_t x, uint8_t y, char *str, const Font *font, OLED_ColorMode color);

// 辅助函数
void OLED_WR_CMD(uint8_t cmd);
void OLED_WR_DATA(uint8_t data);

#endif //TRACKED_OLED_H
