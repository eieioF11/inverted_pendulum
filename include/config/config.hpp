#pragma once
#include <Arduino.h>
#include <M5Unified.h>
#include "utility/dynamixel_utils.hpp"

#define DEBUG_SERIAL Serial
HardwareSerial &DXL_SERIAL = Serial1;

// M5Stack Core2 PORT A
const uint8_t RX_SERVO = 9; // 32
const uint8_t TX_SERVO = 8; // 33

// 右 id
const uint8_t DXL_ID_LW = 1;
const uint8_t DXL_ID_LF = 3;
const uint8_t DXL_ID_LR = 4;
// 左 id
const uint8_t DXL_ID_RW = 2;
const uint8_t DXL_ID_RF = 5;
const uint8_t DXL_ID_RR = 6;

const float DXL_PROTOCOL_VERSION = 2.0;

// const float MAX_RPM = 101.0;// M288
const float MAX_RPM = 370.0; // M077
const float MIN_ANGLE_LIMIT = -10.0 * DEG_TO_RAD;
const float MAX_ANGLE_LIMIT = 90.0 * DEG_TO_RAD;

const float LPF_ALPHA = 0.88;
const float COMP_ALPHA = 0.88;
