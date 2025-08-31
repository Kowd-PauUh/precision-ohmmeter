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

#include "data_aggregation.h"
#include "DFRobot_ADS1115.h"

// ADJUST FOLLOWING AS YOU NEED

// mode switch with hysteresis
constexpr float comparator_th = 1.75;     // 1.75 Ohm (high threshold)
constexpr float comparator_tl = 1.70;     // 1.70 Ohm (low threshold)
constexpr uint8_t mode_control_pin = 29;

// PCB characteristics
constexpr float current = 0.005;                  // 5 mA
constexpr float cell_voltage_divider_gain = 0.5;  // cell voltage divider gain before ADC

// diff. amp. resistors
constexpr float R4 = 10'000.0f;                   // 10 kOhm
constexpr float R5 = 330'000.0f;                  // 330 kOhm
constexpr float R8 = 1'000.0f + 5.0f;             // 1 kOhm + 5 Ohm (typ. ON resistance of the analogue switch)

// ADC pinout
constexpr uint8_t voltage_adc_pin = 0;
constexpr int cell_voltage_adc_pin = 28;

// LCD display
// U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE);   // LCD on 1st I2C interface
U8G2_SSD1306_128X32_UNIVISION_F_2ND_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE);  // LCD on 2nd I2C interface
constexpr uint8_t mode_0_precision = 4;                                   // digits after comma for resistance display in mode 0
constexpr uint8_t mode_1_precision = 3;                                   // digits after comma for resistance display in mode 1

// ADC converter
DFRobot_ADS1115 ads(&Wire);  // ADC on 1st I2C interface
// DFRobot_ADS1115 ads(&Wire1);  // ADC on 2nd I2C interface

// Readings aggregation
constexpr uint8_t buffer_size = 8;
AggregationMode aggregation_mode = AggregationMode::MODE;

// DO NOT MODIFY THESE
constexpr float mode_0_gain = R5 * (R4 + R8) / (R4 * R8);  // diff. amp. gain (363) in first mode
constexpr float mode_1_gain = R5 / R4;                     // diff. amp. gain (33) in second mode
float buffer[buffer_size], scratch[buffer_size];
float voltage, cell_voltage, resistance, gain;
bool buffer_filled = false;
uint8_t precision = mode_0_precision;
uint8_t buffer_index = 0, mode = 0;

void setup() {
    // indicate initialization start
    pinMode(LED_BUILTIN, OUTPUT);
    blinkLed();

    // initialize display
    u8g2.begin();

    // initialize ADC
    ads.setAddr_ADS1115(ADS1115_IIC_ADDRESS0);   // 0x48
    ads.setGain(eGAIN_ONE);                      // no gain
    ads.setMode(eMODE_SINGLE);                   // single-shot mode
    ads.setRate(eRATE_64);                       // 32 samples per second
    ads.setOSMode(eOSMODE_SINGLE);               // manual trigger for each conversion
    ads.init();

    // set pin for mode control 
    pinMode(mode_control_pin, OUTPUT);
}

/**
 * @brief Blinks the built-in LED four times.
 */
void blinkLed() {
    for (uint8_t i = 0; i < 8; i++) {
        if (i % 2 == 0) {
            digitalWrite(LED_BUILTIN, HIGH);
        } else {
            digitalWrite(LED_BUILTIN, LOW);
        }
        delay(250);
    }
}

/**
 * @brief Displays two lines of text on LCD display.
 * @param line1 Text to be written in the first line (font 6x13).
 * @param line2 Text to be written in the second line (font 9x15 bold).
 */
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

/**
 * @brief Computes resistance.
 * 
 * Given voltage drop and current, computes
 * the resistance using Ohm's law:
 *              R = U / I
 * 
 * @param voltage Voltage drop.
 * @param current Current.
 * @return Resistance.
 */
float computeResistance(float voltage, float current) {
    // R = U / I

    if (current == 0.0f) {
        return INFINITY;
    }

    return voltage / current;
}

/**
 * @brief Reads voltage from ADC.
 * 
 * Reads the voltage from the ADC, stores it in a circular buffer,
 * and returns the aggregation result of the most recent samples.
 * 
 * The buffer holds up to `buffer_size` measurements.
 * Before it is full, only valid samples are averaged.
 * Once full, the oldest value is overwritten each new reading.
 * 
 * @return Last `buffer_size` voltage measurements in volts aggregated 
 *         using `aggregation_mode` if ADC is connected. Otherwise NAN.
 */
float readVoltage() {
    float adc_reading = NAN;
    if (ads.checkADS1115())
    {
        adc_reading = ads.readVoltagePrecise(voltage_adc_pin);  // mV
        adc_reading /= 1000;                                    // V
    }

    // store reading in buffer using circular overwrite
    buffer[buffer_index] = adc_reading;
    buffer_index = (buffer_index + 1) % buffer_size;

    // mark buffer filled after full cycle
    if (buffer_index == 0) buffer_filled = true;

    // aggregate readings
    uint8_t count = buffer_filled ? buffer_size : buffer_index;
    return aggregateBuffer(/*buffer=*/buffer, /*count=*/count, /*mode=*/aggregation_mode, /*scratch=*/scratch);
}

/**
 * @brief Reads cell voltage.
 * @return Cell voltage in volts.
 */
float readCellVoltage() {
    return 3.3f * analogRead(cell_voltage_adc_pin) / 1023.0f / cell_voltage_divider_gain;
}

/**
 * @brief Selects gain appropriate to the mode.
 * @param mode Operating mode (0 or 1).
 * @return Gain, defined in `mode_0_gain` and `mode_1_gain` variables. NAN for unsupported mode.
 */
float getGain(uint8_t mode) {
    if (mode == 0) {
        return mode_0_gain;
    } else if (mode == 1) {
        return mode_1_gain;
    }
    return NAN;
}

/**
 * @brief Switches mode with hysteresis.

 * Switches between two modes:
 *     0: `mode_control_pin` is set to LOW state
 *     1: `mode_control_pin` is set to HIGH state
 * 
 * Switch logic is dependent on the current mode and the resistance,
 * and works as a comparator with hysteresis (thresholds values 
 * defined in `comparator_th` and `comparator_tl` variables):
 * 
 * \code{.unparsed}
 *               ↑ Pin state (`mode_control_pin`)
 *               |
 * HIGH (mode 1) |    ←--------        
 *               |   |         ↑
 *               |   ↓         |
 *  LOW (mode 0) |    --------→
 *               +−----------------→  resistance
 *                   tl       th
 * \endcode
 * 
 * @param resistance Input signal to the "comparator".
 */
void switchMode(float resistance) {
    if (mode == 0) {
        if (resistance > comparator_th) { digitalWrite(mode_control_pin, HIGH); mode = 1; precision = mode_1_precision; }
    } else if (mode == 1) {
        if (resistance < comparator_tl) { digitalWrite(mode_control_pin, LOW);  mode = 0; precision = mode_0_precision; }
    }
}

void loop() {
    // read voltage from ADC
    gain = getGain(/*mode=*/mode);
    voltage = readVoltage() / gain;
    voltage = std::max(0.0f, voltage);  // zero out negative reading if occurs
    cell_voltage = readCellVoltage();

    // compute resistance value
    resistance = computeResistance(/*voltage=*/voltage, /*current=*/current);

    // format resistance for displaying
    char resistance_str[15];
    snprintf(resistance_str, sizeof(resistance_str), "%.*f Ohm", precision, resistance);

    // format cell voltage for displaying
    char cell_voltage_str[10];
    snprintf(cell_voltage_str, sizeof(cell_voltage_str), "%.2f V", cell_voltage);

    // display data
    displayText(/*line1=*/cell_voltage_str, /*line2=*/resistance_str);

    // mode switch with hysteresis
    switchMode(/*resistance=*/resistance);
}
