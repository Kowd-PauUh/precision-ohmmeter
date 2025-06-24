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

// ADJUST FOLLOWING AS YOU NEED

constexpr float current = 0.005;                  // 5 mA
constexpr uint8_t adc_bits = 16;                  // 16-bit ADC
constexpr float adc_max_voltage = 3.3;            // 3.3 V
constexpr float cell_voltage_divider_gain = 0.5;  // cell voltage divider gain before ADC

// diff. amp. resistors
constexpr float R4 = 10'000.0f;         // 10 kOhm
constexpr float R5 = 330'000.0f;        // 330 kOhm
constexpr float R8 = 1'000.0f;          // 1 kOhm

// lcd display
// U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);   // LCD on 1st I2C interface
U8G2_SSD1306_128X32_UNIVISION_F_2ND_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);  // LCD on 2nd I2C interface

// DO NOT MODIFY THESE
float cell_voltage, resistance;
constexpr uint32_t adc_range = 1 << adc_bits;
constexpr float mode_0_gain = R5 * (R4 + R8) / (R4 * R8);  // diff. amp. gain in first mode
constexpr float mode_1_gain = R5 / R4;                     // diff. amp. gain in second mode

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

float compute_resistance(uint32_t adc_reading, float current, float diff_amp_gain) {
    // R = U / I

    if (current == 0.0f) {
        return INFINITY;
    }

    float voltage_drop = adc_max_voltage * static_cast<float>(adc_reading) / static_cast<float>(adc_range);
    voltage_drop /= diff_amp_gain;
    return voltage_drop / current;
}

void loop() {
    // compute resistance value
    resistance = compute_resistance(10000, current, mode_0_gain);

    // format resistance for displaying
    char resistance_str[15];
    snprintf(resistance_str, sizeof(resistance_str), "%.5f Ohm", resistance);

    // format cell voltage for displaying
    char cell_voltage_str[10];
    snprintf(cell_voltage_str, sizeof(cell_voltage_str), "%.2f V", cell_voltage);

    // display data
    displayText(cell_voltage_str, resistance_str);
    delay(1000);
    displayText("", "");
    delay(1000);
}
