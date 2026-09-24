#pragma once
#include <Arduino.h>

#include "autopilot/Autopilot.h"
#include "control/FlightController.h"
#include "control/FlightOutputs.h"

// ============================================================
// DEBUG CONSOLE — команды в мониторе порта (буква + Enter)
//
//   h — помощь
//   s — подробный статус всех датчиков (с счётчиками ошибок шины)
//   i — калибровка гироскопа и предполётная проверка IMU (2 с, не двигать)
//   o — калибровка установки IMU: 3 позы, плата может стоять как угодно
//   m — калибровка компаса (15 с, вращать по всем осям)
//   p — самопроверка выходов (реальный импульс на каждом пине)
//
// Калибровки и самопроверка блокируют полётный цикл на время
// выполнения, поэтому разрешены только когда мотор не заармлен.
// ============================================================

class DebugConsole
{
public:

    DebugConsole(FlightController& controller, FlightOutputs& outputs, Autopilot& autopilot)
        : controller(controller),
          outputs(outputs),
          autopilot(autopilot)
    {
    }

    void printHelp() const
    {
        Serial.println("Команды: h — помощь, s — статус датчиков, "
                       "i — калибровка гироскопа (2 с, не двигать), "
                       "o — калибровка установки IMU (3 позы), "
                       "m — калибровка компаса (15 с, вращать по всем осям), "
                       "p — проверка импульсов на выходах");
    }

    // Вызывать каждый цикл: не блокирует, если команды нет.
    void update()
    {
        if (!Serial.available()) return;

        const char command = static_cast<char>(Serial.read());
        if (command == '\r' || command == '\n' || command == ' ') return;

        if (isBlocking(command) && controller.isArmed())
        {
            Serial.println("Консоль: команда недоступна, пока заармлено");
            return;
        }

        switch (command)
        {
            case 's': printSensorStatus(); break;
            case 'i': calibrate(autopilot.getImuSensor(), "IMU"); break;
            case 'o': calibrateImuMounting(); break;
            case 'm': calibrate(autopilot.getMagnetometerSensor(), "компас"); break;
            case 'p': outputs.printPulseSelfTest(); break;
            default:  printHelp(); break;
        }
    }


private:

    FlightController& controller;
    FlightOutputs& outputs;
    Autopilot& autopilot;

    static bool isBlocking(char command)
    {
        return command == 'i' || command == 'o' || command == 'm' || command == 'p';
    }

    void printSensorStatus() const
    {
        const Sensor* sensors[] = {
            autopilot.getImuSensor(),
            autopilot.getBarometerSensor(),
            autopilot.getMagnetometerSensor(),
            autopilot.getGpsSensor(),
        };

        for (const Sensor* sensor : sensors)
        {
            if (sensor) sensor->printStatus();
        }
    }

    void calibrateImuMounting()
    {
        ImuSensor* imu = autopilot.getImuSensor();
        if (!imu)
        {
            Serial.println("Консоль: IMU не выбран в SensorSelection.h");
            return;
        }
        imu->calibrateOrientation();
    }

    template <typename SensorType>
    static void calibrate(SensorType* sensor, const char* what)
    {
        if (!sensor)
        {
            Serial.print("Консоль: ");
            Serial.print(what);
            Serial.println(" не выбран в SensorSelection.h");
            return;
        }
        sensor->calibrate();
    }
};
