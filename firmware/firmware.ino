/*
 * Copyright 2025 Ivan Danylenko
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cstdint>
#include <Wire.h>
#include <U8g2lib.h>
#include "DFRobot_ADS1115.h"

// ADJUST FOLLOWING AS YOU NEED

// mode switch with hysteresis
constexpr float comparator_th = 1.75;     // 1.75 Ohm (high threshold)
constexpr float comparator_tl = 1.70;     // 1.70 Ohm (low threshold)
constexpr uint8_t mode_control_pin = 28;

// PCB characteristics
constexpr float current = 0.005;                  // 5 mA
constexpr float cell_voltage_divider_gain = 0.5;  // cell voltage divider gain before ADC

// diff. amp. resistors
constexpr float R4 = 10'000.0f;                   // 10 kOhm
constexpr float R5 = 330'000.0f;                  // 330 kOhm
constexpr float R8 = 1'000.0f;                    // 1 kOhm

// ADC pinout
constexpr uint8_t voltage_adc_pin = 0;
constexpr uint8_t cell_voltage_adc_pin = 1;

// LCD display
// U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);   // LCD on 1st I2C interface
U8G2_SSD1306_128X32_UNIVISION_F_2ND_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);  // LCD on 2nd I2C interface

// ADC converter
DFRobot_ADS1115 ads(&Wire);  // ADC on 1st I2C interface
// DFRobot_ADS1115 ads(&Wire1);  // ADC on 2nd I2C interface

// DO NOT MODIFY THESE
constexpr float mode_0_gain = R5 * (R4 + R8) / (R4 * R8);  // diff. amp. gain (~363) in first mode
constexpr float mode_1_gain = R5 / R4;                     // diff. amp. gain (33) in second mode
float cell_voltage, voltage, resistance, gain;
uint8_t mode = 0;

void setup() {
    // initialize display
    u8g2.begin();

    // initialize ADC
    ads.setAddr_ADS1115(ADS1115_IIC_ADDRESS0);   // 0x48
    ads.setGain(eGAIN_ONE);                      // no gain
    ads.setMode(eMODE_SINGLE);                   // single-shot mode
    ads.setRate(eRATE_128);                      // 128 samples per second
    ads.setOSMode(eOSMODE_SINGLE);               // manual trigger for each conversion
    ads.init();

    // set pin for mode control 
    pinMode(29, OUTPUT);

    pinMode(29, OUTPUT);     // DEBUG
    digitalWrite(29, HIGH);  // DEBUG
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

float compute_resistance(float voltage, float current) {
    // R = U / I

    if (current == 0.0f) {
        return INFINITY;
    }

    return voltage / current;
}

float readVoltage(uint8_t channel) {
    if (ads.checkADS1115())
    {
        float adc_reading = ads.readVoltagePrecise(channel);  // mV
        adc_reading /= 1000;                                  // V
        return adc_reading;
    }
    return NAN;
}

float getGain(uint8_t mode) {
    if (mode == 0) {
        return mode_0_gain;
    } else if (mode == 1) {
        return mode_1_gain;
    }
    return NAN;
}

void switchMode(float resistance) {
    if (mode == 0) {
        if (resistance > comparator_th) { digitalWrite(mode_control_pin, HIGH); mode = 1; }
    } else if (mode == 1) {
        if (resistance < comparator_tl) { digitalWrite(mode_control_pin, LOW);  mode = 0; }
    }
}

void loop() {
    // read voltage from ADC
    gain = getGain(mode);
    voltage = readVoltage(voltage_adc_pin) / gain;
    cell_voltage = readVoltage(cell_voltage_adc_pin) / cell_voltage_divider_gain;

    // compute resistance value
    resistance = compute_resistance(voltage, current);

    // format resistance for displaying
    char resistance_str[15];
    snprintf(resistance_str, sizeof(resistance_str), "%.5f Ohm", resistance);

    // format cell voltage for displaying
    char cell_voltage_str[10];
    snprintf(cell_voltage_str, sizeof(cell_voltage_str), "%.2f V", cell_voltage);

    // display data
    displayText(cell_voltage_str, resistance_str);

    // mode switch with hysteresis
    switchMode(resistance);
}
