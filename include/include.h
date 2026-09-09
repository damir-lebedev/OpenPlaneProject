#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>

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
#include "../include/sensors/BME280_Sensor.h"
#include "../include/Autopilot.h"
#include "../include/FeatureManager.h"
#include "../include/WebDebugServer.h"

#include "../include/FlightController.h"
#include "../include/DebugLogger.h"