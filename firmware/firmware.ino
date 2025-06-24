/*
Copyright 2025 Ivan Danylenko

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <cstdint>
#include <U8g2lib.h>

// modify these as you need
uint8_t adc_bits = 16;

// do not modify these
uint32_t adc_range = 1 << adc_bits;

// constructor for SSD1306 128x32, I2C, no reset pin
// U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);  // LCD on 1st I2C interface
U8G2_SSD1306_128X32_UNIVISION_F_2ND_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);  // LCD on 2nd I2C interface

void setup() {
    u8g2.begin();  // Initialize display
}

void displayText(const char* line1, const char* line2) {
    u8g2.clearBuffer();

    // 1st line, right-aligned
    u8g2.setFont(u8g2_font_6x13_tf);
    u8g2.drawStr(u8g2.getDisplayWidth() - u8g2.getStrWidth(line1), 12, line1);

    // 2nd line, left-aligned
    u8g2.setFont(u8g2_font_9x15B_tf);
    u8g2.drawStr(0, 28, line2);

    u8g2.sendBuffer();
}

void loop() {
    displayText("0.00 V", "0.0000 Ohm");
    delay(1000);
    displayText("", "");
    delay(1000);
}
