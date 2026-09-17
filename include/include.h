#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>

// HAL — базовые интерфейсы
#include "../include/hal/II2CBus.h"
#include "../include/hal/ISpiBus.h"
#include "../include/hal/IUartPort.h"
#include "../include/hal/IServoOutput.h"
#include "../include/hal/IBoard.h"

// HAL — реализация под ESP32
#include "../include/hal/esp32/Esp32I2CBus.h"
#include "../include/hal/esp32/Esp32SpiBus.h"
#include "../include/hal/esp32/Esp32UartPort.h"
#include "../include/hal/esp32/Esp32ServoOutput.h"
#include "../include/hal/esp32/Esp32Board.h"

#include "../include/Config.h"
#include "../include/Channels.h"
#include "../include/RcChannelState.h"
#include "../include/IBusReceiver.h"
#include "../include/RcInput.h"
#include "../include/FlightOutputState.h"
#include "../include/ControlMixer.h"
#include "../include/ThrottleManager.h"
#include "../include/ArmingManager.h"
#include "../include/FlightOutputs.h"

// Autopilot system (new)
#include "../include/sensors/SensorInterface.h"
#include "../include/sensors/MPU6050_Sensor.h"
#include "../include/sensors/ICM42688_Sensor.h"
#include "../include/sensors/BME280_Sensor.h"
#include "../include/sensors/BMP388_Sensor.h"
#include "../include/sensors/QMC5883P_Sensor.h"
#include "../include/sensors/UbloxM10_Gps.h"
#include "../include/sensors/SensorSelection.h"
#include "../include/Autopilot.h"
#include "../include/FeatureManager.h"
#include "../include/WebDebugServer.h"

#include "../include/FlightController.h"
#include "../include/DebugLogger.h"