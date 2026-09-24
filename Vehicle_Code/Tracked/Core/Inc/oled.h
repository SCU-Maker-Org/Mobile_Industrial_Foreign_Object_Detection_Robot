//
// Created by s on 2026/9/23.
//

#ifndef TRACKED_OLED_H
#define TRACKED_OLED_H

#include "font.h"
#include "string.h"
#include <stdint-gcc.h>
#include "i2c.h"          /* ★ 需要包含 i2c.h，因为要用到 hi2c2 / hi2c1 */

/* ==================================================================== */
/*  ★★★  OLED 使用的 I2C 句柄：改这里就能全局切换 I2C1 / I2C2  ★★★   */
/* ==================================================================== */
/*  当前使用 I2C2（PB10=SCL, PB11=SDA）                                   */
#define OLED_I2C_HANDLE        (&hi2c2)

/*  如果你以后改回 I2C1（PB8=SCL, PB9=SDA），把上面一行注释掉，         */
/*  把下面一行取消注释即可：                                              */
/* #define OLED_I2C_HANDLE     (&hi2c1) */

/* 注意：本驱动使用硬件 I2C，之前软件 I2C 的 GPIO 宏已删除。 */

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