#include "oled.h"
#include "i2c.h"
#include "font.h"
#include <math.h>
#include <stdlib.h>

// OLED器件地址
#define OLED_ADDRESS 0x78

char oled_buffer[32] = {};

// 显存
uint8_t OLED_GRAM[OLED_PAGE][OLED_COLUMN];

// 初始化命令
uint8_t CMD_Data[] = {
        0xAE, 0x00, 0x10, 0x40, 0xB0, 0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x3F,
        0xC8, 0xD3, 0x00, 0xD5, 0x80, 0xD8, 0x05, 0xD9, 0xF1, 0xDA, 0x12,
        0xD8, 0x30, 0x8D, 0x14, 0xAF
};

// ========================== 底层通信函数（I2C版本） ==========================

/**
 * @brief 发送初始化命令序列
 */
void WriteCmd(void)
{
    uint8_t i = 0;
    for(i = 0; i < 27; i++)
    {
        HAL_I2C_Mem_Write(&hi2c1, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, CMD_Data + i, 1, 0x100);
    }
}

/**
 * @brief 向OLED发送指令
 */
void OLED_WR_CMD(uint8_t cmd)
{
    HAL_I2C_Mem_Write(&hi2c1, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 0x100);
}

/**
 * @brief 向OLED发送数据
 */
void OLED_WR_DATA(uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c1, 0x78, 0x40, I2C_MEMADD_SIZE_8BIT, &data, 1, 0x100);
}

/**
 * @brief 发送数据块（用于显示帧）
 */
void OLED_Send(uint8_t *data, uint8_t len)
{
    HAL_I2C_Mem_Write(&hi2c1, 0x78, 0x40, I2C_MEMADD_SIZE_8BIT, data, len, 0x100);
}

// ========================== OLED驱动函数 ==========================

/**
 * @brief 初始化OLED (SSD1306)
 */
void OLED_Init()
{
    HAL_Delay(200);
    WriteCmd();

    // 初始化显存
    OLED_NewFrame();
    OLED_ShowFrame();

    // 开启显示
    OLED_DisPlay_On();
}

/**
 * @brief 开启OLED显示
 */
void OLED_DisPlay_On()
{
    OLED_WR_CMD(0X8D);
    OLED_WR_CMD(0X14);
    OLED_WR_CMD(0XAF);
}

/**
 * @brief 关闭OLED显示
 */
void OLED_DisPlay_Off()
{
    OLED_WR_CMD(0X8D);
    OLED_WR_CMD(0X10);
    OLED_WR_CMD(0XAE);
}

/**
 * @brief 设置颜色模式 黑底白字或白底黑字
 */
void OLED_SetColorMode(OLED_ColorMode mode)
{
    if (mode == OLED_COLOR_NORMAL)
    {
        OLED_WR_CMD(0xA6); // 正常显示
    }
    else if (mode == OLED_COLOR_REVERSED)
    {
        OLED_WR_CMD(0xA7); // 反色显示
    }
}

// ========================== 显存操作函数 ==========================

/**
 * @brief 清空显存 绘制新的一帧
 */
void OLED_NewFrame()
{
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
}

/**
 * @brief 将当前显存显示到屏幕上
 */
void OLED_ShowFrame()
{
    static uint8_t sendBuffer[OLED_COLUMN + 1];

    for (uint8_t i = 0; i < OLED_PAGE; i++)
    {
        OLED_WR_CMD(0xB0 + i); // 设置页地址
        OLED_WR_CMD(0x00);     // 设置列地址低4位
        OLED_WR_CMD(0x10);     // 设置列地址高4位

        // 发送该页的所有列数据
        for (uint8_t j = 0; j < OLED_COLUMN; j++)
        {
            OLED_WR_DATA(OLED_GRAM[i][j]);
        }
    }
}

/**
 * @brief 设置一个像素点
 */
void OLED_SetPixel(uint8_t x, uint8_t y, OLED_ColorMode color)
{
    if (x >= OLED_COLUMN || y >= OLED_ROW)
        return;

    if (color == OLED_COLOR_NORMAL) // 正常模式：黑底白字，设置像素点为1（亮）
    {
        OLED_GRAM[y / 8][x] |= 1 << (y % 8);
    }
    else // 反色模式：白底黑字，设置像素点为0（暗）
    {
        OLED_GRAM[y / 8][x] &= ~(1 << (y % 8));
    }
}

/**
 * @brief 设置显存中的一字节数据
 */
static void OLED_SetByte(uint8_t page, uint8_t column, uint8_t data, OLED_ColorMode color)
{
    if (page >= OLED_PAGE || column >= OLED_COLUMN)
        return;

    if (color == OLED_COLOR_REVERSED)
        data = ~data;

    OLED_GRAM[page][column] = data;
}

/**
 * @brief 设置显存中一字节数据的某几位
 */
static void OLED_SetByte_Fine(uint8_t page, uint8_t column, uint8_t data, uint8_t start, uint8_t end, OLED_ColorMode color)
{
    uint8_t temp;
    if (page >= OLED_PAGE || column >= OLED_COLUMN)
        return;

    if (color == OLED_COLOR_REVERSED)
        data = ~data;

    temp = data | (0xFF << (end + 1)) | (0xFF >> (8 - start));
    OLED_GRAM[page][column] &= temp;
    temp = data & ~(0xFF << (end + 1)) & ~(0xFF >> (8 - start));
    OLED_GRAM[page][column] |= temp;
}

/**
 * @brief 设置显存中的一字节长度的数据
 */
static void OLED_SetBits(uint8_t x, uint8_t y, uint8_t data, OLED_ColorMode color)
{
    uint8_t page = y / 8;
    uint8_t bit = y % 8;

    OLED_SetByte_Fine(page, x, data << bit, bit, 7, color);
    if (bit)
    {
        OLED_SetByte_Fine(page + 1, x, data >> (8 - bit), 0, bit - 1, color);
    }
}

/**
 * @brief 设置显存中的一字节数据的某几位
 */
static void OLED_SetBits_Fine(uint8_t x, uint8_t y, uint8_t data, uint8_t len, OLED_ColorMode color)
{
    uint8_t page = y / 8;
    uint8_t bit = y % 8;

    if (bit + len > 8)
    {
        OLED_SetByte_Fine(page, x, data << bit, bit, 7, color);
        OLED_SetByte_Fine(page + 1, x, data >> (8 - bit), 0, len + bit - 1 - 8, color);
    }
    else
    {
        OLED_SetByte_Fine(page, x, data << bit, bit, bit + len - 1, color);
    }
}

/**
 * @brief 设置一块显存区域
 */
static void OLED_SetBlock(uint8_t x, uint8_t y, const uint8_t *data, uint8_t w, uint8_t h, OLED_ColorMode color)
{
    uint8_t fullRow = h / 8; // 完整的行数
    uint8_t partBit = h % 8; // 不完整的字节中的有效位数

    for (uint8_t i = 0; i < w; i++)
    {
        for (uint8_t j = 0; j < fullRow; j++)
        {
            OLED_SetBits(x + i, y + j * 8, data[i + j * w], color);
        }
    }

    if (partBit)
    {
        uint16_t fullNum = w * fullRow; // 完整的字节数
        for (uint8_t i = 0; i < w; i++)
        {
            OLED_SetBits_Fine(x + i, y + (fullRow * 8), data[fullNum + i], partBit, color);
        }
    }
}

// ========================== 图形绘制函数 ==========================

/**
 * @brief 绘制一条线段
 */
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, OLED_ColorMode color)
{
    uint8_t temp = 0;
    if (x1 == x2)
    {
        if (y1 > y2)
        {
            temp = y1;
            y1 = y2;
            y2 = temp;
        }
        for (uint8_t y = y1; y <= y2; y++)
        {
            OLED_SetPixel(x1, y, color);
        }
    }
    else if (y1 == y2)
    {
        if (x1 > x2)
        {
            temp = x1;
            x1 = x2;
            x2 = temp;
        }
        for (uint8_t x = x1; x <= x2; x++)
        {
            OLED_SetPixel(x, y1, color);
        }
    }
    else
    {
        // Bresenham直线算法
        int16_t dx = x2 - x1;
        int16_t dy = y2 - y1;
        int16_t ux = ((dx > 0) << 1) - 1;
        int16_t uy = ((dy > 0) << 1) - 1;
        int16_t x = x1, y = y1, eps = 0;
        dx = abs(dx);
        dy = abs(dy);

        if (dx > dy)
        {
            for (x = x1; x != x2; x += ux)
            {
                OLED_SetPixel(x, y, color);
                eps += dy;
                if ((eps << 1) >= dx)
                {
                    y += uy;
                    eps -= dx;
                }
            }
        }
        else
        {
            for (y = y1; y != y2; y += uy)
            {
                OLED_SetPixel(x, y, color);
                eps += dx;
                if ((eps << 1) >= dy)
                {
                    x += ux;
                    eps -= dy;
                }
            }
        }
    }
}

/**
 * @brief 绘制一个矩形
 */
void OLED_DrawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode color)
{
    OLED_DrawLine(x, y, x + w, y, color);
    OLED_DrawLine(x, y + h, x + w, y + h, color);
    OLED_DrawLine(x, y, x, y + h, color);
    OLED_DrawLine(x + w, y, x + w, y + h, color);
}

/**
 * @brief 绘制一个填充矩形
 */
void OLED_DrawFilledRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode color)
{
    for (uint8_t i = 0; i < h; i++)
    {
        OLED_DrawLine(x, y + i, x + w, y + i, color);
    }
}

/**
 * @brief 绘制一个三角形
 */
void OLED_DrawTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, OLED_ColorMode color)
{
    OLED_DrawLine(x1, y1, x2, y2, color);
    OLED_DrawLine(x2, y2, x3, y3, color);
    OLED_DrawLine(x3, y3, x1, y1, color);
}

/**
 * @brief 绘制一个填充三角形
 */
void OLED_DrawFilledTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, OLED_ColorMode color)
{
    uint8_t a = 0, b = 0, y_pos = 0, last = 0;

    if (y1 > y2)
    {
        a = y2;
        b = y1;
    }
    else
    {
        a = y1;
        b = y2;
    }

    y_pos = a;
    for (; y_pos <= b; y_pos++)
    {
        if (y_pos <= y3)
        {
            OLED_DrawLine(x1 + (y_pos - y1) * (x2 - x1) / (y2 - y1), y_pos,
                          x1 + (y_pos - y1) * (x3 - x1) / (y3 - y1), y_pos, color);
        }
        else
        {
            last = y_pos - 1;
            break;
        }
    }

    for (; y_pos <= b; y_pos++)
    {
        OLED_DrawLine(x2 + (y_pos - y2) * (x3 - x2) / (y3 - y2), y_pos,
                      x1 + (y_pos - last) * (x3 - x1) / (y3 - last), y_pos, color);
    }
}

/**
 * @brief 绘制一个圆
 */
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r, OLED_ColorMode color)
{
    int16_t a = 0, b = r, di = 3 - (r << 1);

    while (a <= b)
    {
        OLED_SetPixel(x - b, y - a, color);
        OLED_SetPixel(x + b, y - a, color);
        OLED_SetPixel(x - a, y + b, color);
        OLED_SetPixel(x - b, y - a, color);
        OLED_SetPixel(x - a, y - b, color);
        OLED_SetPixel(x + b, y + a, color);
        OLED_SetPixel(x + a, y - b, color);
        OLED_SetPixel(x + a, y + b, color);
        OLED_SetPixel(x - b, y + a, color);

        a++;
        if (di < 0)
        {
            di += 4 * a + 6;
        }
        else
        {
            di += 10 + 4 * (a - b);
            b--;
        }
        OLED_SetPixel(x + a, y + b, color);
    }
}

/**
 * @brief 绘制一个填充圆
 */
void OLED_DrawFilledCircle(uint8_t x, uint8_t y, uint8_t r, OLED_ColorMode color)
{
    int16_t a = 0, b = r, di = 3 - (r << 1);

    while (a <= b)
    {
        for (int16_t i = x - b; i <= x + b; i++)
        {
            OLED_SetPixel(i, y + a, color);
            OLED_SetPixel(i, y - a, color);
        }
        for (int16_t i = x - a; i <= x + a; i++)
        {
            OLED_SetPixel(i, y + b, color);
            OLED_SetPixel(i, y - b, color);
        }

        a++;
        if (di < 0)
        {
            di += 4 * a + 6;
        }
        else
        {
            di += 10 + 4 * (a - b);
            b--;
        }
    }
}

/**
 * @brief 绘制一个椭圆
 */
void OLED_DrawEllipse(uint8_t x, uint8_t y, uint8_t a, uint8_t b, OLED_ColorMode color)
{
    int xpos = 0, ypos = b;
    int a2 = a * a, b2 = b * b;
    int d = b2 + a2 * (0.25 - b);

    while (a2 * ypos > b2 * xpos)
    {
        OLED_SetPixel(x + xpos, y + ypos, color);
        OLED_SetPixel(x - xpos, y + ypos, color);
        OLED_SetPixel(x + xpos, y - ypos, color);
        OLED_SetPixel(x - xpos, y - ypos, color);

        if (d < 0)
        {
            d = d + b2 * ((xpos << 1) + 3);
            xpos += 1;
        }
        else
        {
            d = d + b2 * ((xpos << 1) + 3) + a2 * (-(ypos << 1) + 2);
            xpos += 1;
            ypos -= 1;
        }
    }

    d = b2 * (xpos + 0.5) * (xpos + 0.5) + a2 * (ypos - 1) * (ypos - 1) - a2 * b2;

    while (ypos > 0)
    {
        OLED_SetPixel(x + xpos, y + ypos, color);
        OLED_SetPixel(x - xpos, y + ypos, color);
        OLED_SetPixel(x + xpos, y - ypos, color);
        OLED_SetPixel(x - xpos, y - ypos, color);

        if (d < 0)
        {
            d = d + b2 * ((xpos << 1) + 2) + a2 * (-(ypos << 1) + 3);
            xpos += 1;
            ypos -= 1;
        }
        else
        {
            d = d + a2 * (-(ypos << 1) + 3);
            ypos -= 1;
        }
    }
}

/**
 * @brief 绘制一张图片
 */
void OLED_DrawImage(uint8_t x, uint8_t y, const Image *img, OLED_ColorMode color)
{
    OLED_SetBlock(x, y, img->data, img->w, img->h, color);
}

// ========================== 文字绘制函数 ==========================

/**
 * @brief 绘制一个ASCII字符
 */
void OLED_PrintASCIIChar(uint8_t x, uint8_t y, char ch, const ASCIIFont *font, OLED_ColorMode color)
{
    OLED_SetBlock(x, y, font->chars + (ch - ' ') * (((font->h + 7) / 8) * font->w), font->w, font->h, color);
}

/**
 * @brief 绘制一个ASCII字符串
 */
void OLED_PrintASCIIString(uint8_t x, uint8_t y, char *str, const ASCIIFont *font, OLED_ColorMode color)
{
    uint8_t x0 = x;
    while (*str)
    {
        OLED_PrintASCIIChar(x0, y, *str, font, color);
        x0 += font->w;
        str++;
    }
}

/**
 * @brief 获取UTF-8编码的字符长度
 */
static uint8_t _OLED_GetUTF8Len(char *string)
{
    if ((string[0] & 0x80) == 0x00)
    {
        return 1;
    }
    else if ((string[0] & 0xE0) == 0xC0)
    {
        return 2;
    }
    else if ((string[0] & 0xF0) == 0xE0)
    {
        return 3;
    }
    else if ((string[0] & 0xF8) == 0xF0)
    {
        return 4;
    }
    return 0;
}

/**
 * @brief 绘制字符串（支持中英文混合）
 */
void OLED_PrintString(uint8_t x, uint8_t y, char *str, const Font *font, OLED_ColorMode color)
{
    uint16_t i = 0;                                       // 字符串索引
    uint8_t oneLen = (((font->h + 7) / 8) * font->w) + 4; // 一个字模占多少字节
    uint8_t found;                                        // 是否找到字模
    uint8_t utf8Len;                                      // UTF-8编码长度
    uint8_t *head;                                        // 字模头指针

    while (str[i])
    {
        found = 0;
        utf8Len = _OLED_GetUTF8Len(str + i);
        if (utf8Len == 0)
            break; // 有问题的UTF-8编码

        // 寻找字符
        for (uint8_t j = 0; j < font->len; j++)
        {
            head = (uint8_t *)(font->chars) + (j * oneLen);
            if (memcmp(str + i, head, utf8Len) == 0)
            {
                OLED_SetBlock(x, y, head + 4, font->w, font->h, color);
                // 移动光标
                x += font->w;
                i += utf8Len;
                found = 1;
                break;
            }
        }

        // 若未找到字模,且为ASCII字符, 则缺省显示ASCII字符
        if (found == 0)
        {
            if (utf8Len == 1)
            {
                OLED_PrintASCIIChar(x, y, str[i], font->ascii, color);
                // 移动光标
                x += font->ascii->w;
                i += utf8Len;
            }
            else
            {
                OLED_PrintASCIIChar(x, y, ' ', font->ascii, color);
                x += font->ascii->w;
                i += utf8Len;
            }
        }
    }
}