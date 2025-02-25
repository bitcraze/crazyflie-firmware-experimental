/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2024 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * led_control.c - Control the HP-RGBW LED
 *
 */

 #include "led_control.h"
 #include "param.h"

 paramVarId_t idLEDDeck;

 const uint8_t gamma8[256] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

 static float smallest(float x, float y, float z)
{
  return x < y ? (x < z ? x : z) : (y < z ? y : z);
}

static uint8_t limitUint8(int32_t value)
{
  if(value > UINT8_MAX)
  {
    value = UINT8_MAX;
  }
  else if(value < 0)
  {
    value = 0;
  }

  return (uint8_t)value;
}

void ledControlInit(void)
{
    idLEDDeck = paramGetVarId("hprgbw", "rgbw8888");
    paramSetInt(idLEDDeck, 0x00000000); 
}

void ledSetRGB(uint8_t r, uint8_t g, uint8_t b)
{
    paramSetInt(idLEDDeck, (r << 24) | (g << 16) | (b << 8)); 
}

void ledSetRGBW(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    paramSetInt(idLEDDeck, (r << 24) | (g << 16) | (b << 8) | w); 
}

void ledSetColorFromXYZ(float x, float y, float z)
{
    uint8_t r, g, b;
    float rf, gf, bf;
    float removeWhite;

    rf = (x + 1.5f) * 255 / 3;
    gf = (y + 1.5f) * 255 / 3;
    bf = (z + 0.0f) * 255 / 3;

    removeWhite = smallest(rf, gf, bf);

    // Remove most of the white
    rf = rf - (removeWhite * 0.8f);
    gf = gf - (removeWhite * 0.8f);
    bf = bf - (removeWhite * 0.8f);

    // Scale to sum 255 and make sure no overflow
    r = limitUint8(rf * 255.0f / (rf + gf + bf));
    g = limitUint8(gf * 255.0f / (rf + gf + bf));
    b = limitUint8(bf * 255.0f / (rf + gf + bf));

    ledSetRGB(gamma8[r],  gamma8[g],  gamma8[b]);
}