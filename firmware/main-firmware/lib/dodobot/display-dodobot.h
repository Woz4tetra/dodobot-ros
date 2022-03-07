
#ifndef __DODOBOT_DISPLAY_H__
#define __DODOBOT_DISPLAY_H__

#include <Arduino.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>
#include <math.h>

#include "dodobot.h"


/*
 * Adafruit TFT 1.8" display, 160x128
 * ST7735
 */

#define TFT_CS    10
#define TFT_RST    9
#define TFT_DC     8
#define TFT_LITE   6


#define  TO_RADIANS(x)  x * PI / 180.0


#define ST77XX_GRAY          0x8410
#define ST77XX_LIGHT_PINK    0xfcd3
#define ST77XX_LIGHT_BLUE    0xcedf
#define ST77XX_DARKER_BLUE   0x3bdb
#define ST77XX_DARKER_GREEN  0x0680
#define ST77XX_DARK_GREEN    0x06e0
#define ST77XX_ORANGE        0xFC00


namespace dodobot_display
{
    const float TFT_PI = 3.1415926;
    uint32_t tft_display_timer = 0;

    Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);
    uint8_t tft_brightness;

    void set_display_brightness(int brightness)
    {
        if (tft_brightness == brightness) {
            return;
        }
        analogWrite(TFT_LITE, brightness);
        tft_brightness = brightness;
    }

    void black_display() {
        tft.fillScreen(ST77XX_BLACK);
    }

    void setup_display()
    {
        pinMode(TFT_LITE, OUTPUT);
        tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
        delay(10);
        set_display_brightness(255);
        black_display();
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

        tft.setTextWrap(false);
        tft.setTextSize(1);
        tft.setRotation(3); // horizontal display

        tft.print("Hello!\n");
        dodobot_serial::println_info("Display ready");
    }

    void textBounds(String s, uint16_t& w, uint16_t& h) {
        int16_t x1, y1;
        tft.getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
    }

    void fillPrevText(int16_t prev_x, int16_t prev_y, uint16_t prev_w, uint16_t prev_h,
                      int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t bg_color)
    {
        // fill rectangles on both sides of the previous text:
        // |xx|        |xx|
        // |xx|        |xx|
        // |xx|        |xx|
        if (prev_x - x < 0) {
            tft.fillRect(prev_x, prev_y, x - prev_x, prev_h, bg_color);
        }
        if (((prev_x + prev_w) - (x + w)) > 0) {
            tft.fillRect(x + w, prev_y, (prev_x + prev_w) - (x + w), prev_h, bg_color);
        }

        // fill rectangles on top and bottom of the previous text
        // |  |xxxxxxxx|  |
        // |  |        |  |
        // |  |xxxxxxxx|  |
        if (prev_y - y < 0) {
            tft.fillRect(x, prev_y, w, y - prev_y, bg_color);
        }
        if ((prev_y + prev_h) - (y + h) > 0) {
            tft.fillRect(x, y + h, w, (prev_y + prev_h) - (y + h), bg_color);
        }
    }


    void drawArc(int x, int y, int start_angle, int stop_angle, int rx, int ry, int w, uint16_t color, int increment = 3)
    {
        double sx = cos(TO_RADIANS(start_angle));
        double sy = sin(TO_RADIANS(start_angle));
        double sx2, sy2;

        double x0 = round(sx * (rx - w) + x);
        double y0 = round(sy * (ry - w) + y);
        double x1 = round(sx * rx + x);
        double y1 = round(sy * ry + y);
        double x2, y2, x3, y3;
        int i = start_angle;
        while (true)
        {
            if (stop_angle >= start_angle)
            {
                i += increment;
                if (i > stop_angle) {
                    break;
                }
            }
            else if (stop_angle <= start_angle)
            {
                i -= increment;
                if (i < stop_angle) {
                    break;
                }
            }

            sx2 = cos(TO_RADIANS(i));
            sy2 = sin(TO_RADIANS(i));

            x2 = round(sx2 * (rx - w) + x);
            y2 = round(sy2 * (ry - w) + y);
            x3 = round(sx2 * rx + x);
            y3 = round(sy2 * ry + y);

            tft.fillTriangle(x0, y0, x1, y1, x2, y2, color);
            tft.fillTriangle(x1, y1, x2, y2, x3, y3, color);

            x0 = x2;
            y0 = y2;
            x1 = x3;
            y1 = y3;
        }
    }

    void fillArc(int x, int y, int start_angle, int stop_angle, int rx, int ry, uint16_t color, int increment = 3)
    {
        double sx = cos(TO_RADIANS(start_angle));
        double sy = sin(TO_RADIANS(start_angle));
        double sx2, sy2;

        int x0 = (int)round(x);
        int y0 = (int)round(y);
        int x1 = (int)round(sx * rx + x);
        int y1 = (int)round(sy * ry + y);
        int x2, y2;
        int i = start_angle;
        while (true)
        {
            if (stop_angle >= start_angle)
            {
                i += increment;
                if (i > stop_angle) {
                    break;
                }
            }
            else if (stop_angle <= start_angle)
            {
                i -= increment;
                if (i < stop_angle) {
                    break;
                }
            }

            sx2 = cos(TO_RADIANS(i));
            sy2 = sin(TO_RADIANS(i));

            x2 = (int)round(sx2 * rx + x);
            y2 = (int)round(sy2 * ry + y);

            tft.fillTriangle(x0, y0, x1, y1, x2, y2, color);

            x1 = x2;
            y1 = y2;
        }
    }


    void drawCircle(int16_t x0, int16_t y0, uint16_t r, uint16_t w, uint16_t color)
    {
        // for (int16_t r_index = -w; r_index <= 0; r_index++) {
        //     tft.drawCircle(x0, y0, r + r_index, color);
        // }
        drawArc(x0, y0, 0, 360, r, r, w, color);
    }
};

#endif  // __DODOBOT_DISPLAY_H__
