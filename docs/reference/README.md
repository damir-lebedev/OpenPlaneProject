# Справочник по классам

Полный перечень классов, структур, перечислений и пространств имён прошивки,
сгруппированный по слоям. Общая картина (слои, потоки, автоматы) — в
[`../ARCHITECTURE.md`](../ARCHITECTURE.md); практические рецепты — в
[`../DEVELOPER_GUIDE.md`](../DEVELOPER_GUIDE.md).

Все пути — от `include/`, кроме `src/main.cpp`. Все классы header-only.

| Страница | Слой | Что внутри |
|---|---|---|
| [config.md](config.md) | CONFIG | `Config`, `Channels` |
| [hal.md](hal.md) | HAL | `IBoard`, `ServoChannel`, `II2CBus`, `ISpiBus`, `IUartPort`, `IServoOutput`, `IRegisterDevice`, `I2cRegisterDevice`, `SpiRegisterDevice`, `Esp32Board`, `Esp32I2CBus`, `Esp32SpiBus`, `Esp32UartPort`, `Esp32ServoOutput`; заготовка STM32H743: `Stm32Board`, `Stm32I2CBus`, `Stm32SpiBus`, `Stm32UartPort`, `Stm32ServoOutput` |
| [rc.md](rc.md) | RC | `RcChannelState`, `RcInput`, `IBusReceiver` |
| [control.md](control.md) | CONTROL / COORDINATION | `ControlCommand`, `FlightOutputState`, `FlapsController`, `ControlMixer`, `ThrottleManager`, `ArmingManager`, `FlightOutputs`, `FlightController` |
| [autopilot.md](autopilot.md) | AUTOPILOT | `AutopilotMode`, `PidController`, `Autopilot`, `AutopilotModeSelector` |
| [feedback.md](feedback.md) | AUTOPILOT / feedback | `FeedbackConfig`, `FeedbackMath`, `FlightSnapshot`, `FeedbackOutput`, `PhaseTargets`, `SpeedEstimator`, `AirborneDetector`, `ControlEffectivenessEstimator`, `AxisModel`, `AdaptiveRateController`, `StallGuard`, `TakeoffSequencer`, `LandingSequencer`, `FeedbackSupervisor` |
| [sensors.md](sensors.md) | SENSORS | интерфейсы и данные, `SensorMounting`, `SensorSelection`, `ImuOrientation`, `AttitudeEstimator`, `ImuSensorBase`, `MPU6050_Sensor`, `ICM42688_Sensor`, `BarometerBase`, `BMP388_Sensor`, `BME280_Sensor`, `MagnetometerBase`, `QMC5883P_Sensor`, `QMC5883L_Sensor`, `UbloxM10_Gps`, `AirspeedSensor` |
| [telemetry.md](telemetry.md) | TELEMETRY | `LoopStats`, `LogChannel`, `LogMode`, `LogSettings`, `DebugLogger`, `DebugConsole`, `WebDashboardPage`, `WebDebugServer`, `OledDisplay` |
| [application.md](application.md) | APPLICATION | `src/main.cpp`: глобальные объекты, `setup()`, `loop()`; `src/stm32/main.cpp` — bring-up STM32H743 |

## Алфавитный указатель

| Сущность | Вид | Файл | Страница |
|---|---|---|---|
| `AdaptiveRateController` | class | `autopilot/feedback/AdaptiveRateController.h` | [feedback](feedback.md#adaptiveratecontroller) |
| `AirborneDetector` | class | `autopilot/feedback/AirborneDetector.h` | [feedback](feedback.md#airbornedetector) |
| `AirspeedData`, `AirspeedSensor` | struct, interface | `sensors/airspeed/AirspeedSensor.h` | [sensors](sensors.md#airspeedsensor) |
| `ArmingManager` | class | `control/ArmingManager.h` | [control](control.md#armingmanager) |
| `AttitudeEstimator` | class | `sensors/imu/AttitudeEstimator.h` | [sensors](sensors.md#attitudeestimator) |
| `Autopilot` | class | `autopilot/Autopilot.h` | [autopilot](autopilot.md#autopilot) |
| `AutopilotMode` | enum | `autopilot/Autopilot.h` | [autopilot](autopilot.md#autopilotmode) |
| `AutopilotModeSelector` | class | `autopilot/AutopilotModeSelector.h` | [autopilot](autopilot.md#autopilotmodeselector) |
| `AxisModel` | struct | `autopilot/feedback/AdaptiveRateController.h` | [feedback](feedback.md#axismodel) |
| `BarometerBase` | abstract class | `sensors/baro/BarometerBase.h` | [sensors](sensors.md#barometerbase) |
| `BarometerData`, `BarometerSensor` | struct, interface | `sensors/SensorInterface.h` | [sensors](sensors.md#интерфейсы-и-структуры-данных) |
| `BME280_Sensor` | class | `sensors/baro/BME280_Sensor.h` | [sensors](sensors.md#bme280_sensor) |
| `BMP388_Sensor` | class | `sensors/baro/BMP388_Sensor.h` | [sensors](sensors.md#bmp388_sensor) |
| `Channels` | namespace | `config/Channels.h` | [config](config.md#namespace-channels) |
| `Config` | namespace | `config/Config.h` | [config](config.md#namespace-config) |
| `ControlCommand` | struct | `control/ControlCommand.h` | [control](control.md#controlcommand) |
| `ControlEffectivenessEstimator` | class | `autopilot/feedback/ControlEffectivenessEstimator.h` | [feedback](feedback.md#controleffectivenessestimator) |
| `ControlMixer` | class | `control/ControlMixer.h` | [control](control.md#controlmixer) |
| `DebugConsole` | class | `telemetry/DebugConsole.h` | [telemetry](telemetry.md#debugconsole) |
| `DebugLogger` | class | `telemetry/DebugLogger.h` | [telemetry](telemetry.md#debuglogger) |
| `Esp32Board` | class | `hal/esp32/Esp32Board.h` | [hal](hal.md#esp32board) |
| `Esp32I2CBus` | class | `hal/esp32/Esp32I2CBus.h` | [hal](hal.md#esp32i2cbus) |
| `Esp32ServoOutput` | class | `hal/esp32/Esp32ServoOutput.h` | [hal](hal.md#esp32servooutput) |
| `Esp32SpiBus` | class | `hal/esp32/Esp32SpiBus.h` | [hal](hal.md#esp32spibus) |
| `Esp32UartPort` | class | `hal/esp32/Esp32UartPort.h` | [hal](hal.md#esp32uartport) |
| `FeedbackConfig` | namespace | `autopilot/feedback/FeedbackConfig.h` | [feedback](feedback.md#namespace-feedbackconfig) |
| `FeedbackMath` | namespace | `autopilot/feedback/FeedbackMath.h` | [feedback](feedback.md#namespace-feedbackmath) |
| `FeedbackOutput` | struct | `autopilot/feedback/FeedbackOutput.h` | [feedback](feedback.md#feedbackoutput) |
| `FeedbackSupervisor` | class | `autopilot/feedback/FeedbackSupervisor.h` | [feedback](feedback.md#feedbacksupervisor) |
| `FlapsController` | class | `control/FlapsController.h` | [control](control.md#flapscontroller) |
| `FlightController` | class | `control/FlightController.h` | [control](control.md#flightcontroller) |
| `FlightOutputs`, `FlightOutputs::OutputInfo` | class, struct | `control/FlightOutputs.h` | [control](control.md#flightoutputs) |
| `FlightOutputState` | struct | `control/FlightOutputState.h` | [control](control.md#flightoutputstate) |
| `FlightSnapshot` | struct | `autopilot/feedback/FlightSnapshot.h` | [feedback](feedback.md#flightsnapshot) |
| `GpsData`, `GpsSensor` | struct, interface | `sensors/SensorInterface.h` | [sensors](sensors.md#интерфейсы-и-структуры-данных) |
| `I2cRegisterDevice` | class | `hal/RegisterDevice.h` | [hal](hal.md#i2cregisterdevice) |
| `IBoard` | interface | `hal/IBoard.h` | [hal](hal.md#iboard) |
| `IBusReceiver` | class | `rc/IBusReceiver.h` | [rc](rc.md#ibusreceiver) |
| `ICM42688_Sensor` | class | `sensors/imu/ICM42688_Sensor.h` | [sensors](sensors.md#icm42688_sensor) |
| `II2CBus` | interface | `hal/II2CBus.h` | [hal](hal.md#ii2cbus) |
| `ImuData`, `ImuSensor` | struct, interface | `sensors/SensorInterface.h` | [sensors](sensors.md#интерфейсы-и-структуры-данных) |
| `ImuOrientation` | class | `sensors/imu/ImuOrientation.h` | [sensors](sensors.md#imuorientation) |
| `ImuSensorBase` | abstract class | `sensors/imu/ImuSensorBase.h` | [sensors](sensors.md#imusensorbase) |
| `IRegisterDevice` | interface | `hal/RegisterDevice.h` | [hal](hal.md#iregisterdevice) |
| `IServoOutput` | interface | `hal/IServoOutput.h` | [hal](hal.md#iservooutput) |
| `ISpiBus` | interface | `hal/ISpiBus.h` | [hal](hal.md#ispibus) |
| `IUartPort` | interface | `hal/IUartPort.h` | [hal](hal.md#iuartport) |
| `LandingSequencer` | class | `autopilot/feedback/LandingSequencer.h` | [feedback](feedback.md#landingsequencer) |
| `LogChannel`, `LogMode`, `LogChannelInfo` | enum, enum, struct | `telemetry/LogSettings.h` | [telemetry](telemetry.md#logsettings) |
| `LogSettings` | class | `telemetry/LogSettings.h` | [telemetry](telemetry.md#logsettings) |
| `LoopStats` | struct | `telemetry/LoopStats.h` | [telemetry](telemetry.md#loopstats) |
| `MagData`, `MagnetometerSensor` | struct, interface | `sensors/SensorInterface.h` | [sensors](sensors.md#интерфейсы-и-структуры-данных) |
| `MagnetometerBase` | abstract class | `sensors/mag/MagnetometerBase.h` | [sensors](sensors.md#magnetometerbase) |
| `MPU6050_Sensor` | class | `sensors/imu/MPU6050_Sensor.h` | [sensors](sensors.md#mpu6050_sensor) |
| `OledDisplay` | class | `telemetry/OledDisplay.h` | [telemetry](telemetry.md#oleddisplay) |
| `PhaseTargets` | struct | `autopilot/feedback/PhaseTargets.h` | [feedback](feedback.md#phasetargets) |
| `PidController` | class | `autopilot/PidController.h` | [autopilot](autopilot.md#pidcontroller) |
| `QMC5883L_Sensor` | class | `sensors/mag/QMC5883L_Sensor.h` | [sensors](sensors.md#qmc5883l_sensor) |
| `QMC5883P_Sensor` | class | `sensors/mag/QMC5883P_Sensor.h` | [sensors](sensors.md#qmc5883p_sensor) |
| `RawImuSample` | struct | `sensors/imu/ImuSensorBase.h` | [sensors](sensors.md#imusensorbase) |
| `RcChannelState` | class | `rc/RcChannelState.h` | [rc](rc.md#rcchannelstate) |
| `RcInput` | class (static) | `rc/RcInput.h` | [rc](rc.md#rcinput) |
| `Sensor` | interface | `sensors/SensorInterface.h` | [sensors](sensors.md#интерфейсы-и-структуры-данных) |
| `SensorMounting` | namespace | `sensors/SensorMounting.h` | [sensors](sensors.md#namespace-sensormounting) |
| `SensorSelection.h` | макросы | `sensors/SensorSelection.h` | [sensors](sensors.md#sensorselectionh) |
| `ServoChannel` | namespace | `hal/IBoard.h` | [hal](hal.md#namespace-servochannel) |
| `SpeedEstimator` | class | `autopilot/feedback/SpeedEstimator.h` | [feedback](feedback.md#speedestimator) |
| `SpiRegisterDevice` | class | `hal/RegisterDevice.h` | [hal](hal.md#spiregisterdevice) |
| `StallGuard` | class | `autopilot/feedback/StallGuard.h` | [feedback](feedback.md#stallguard) |
| `Stm32Board` | class | `hal/stm32/Stm32Board.h` | [hal](hal.md#stm32board) |
| `Stm32I2CBus` | class | `hal/stm32/Stm32I2CBus.h` | [hal](hal.md#stm32i2cbus) |
| `Stm32ServoOutput` | class | `hal/stm32/Stm32ServoOutput.h` | [hal](hal.md#stm32servooutput) |
| `Stm32SpiBus` | class | `hal/stm32/Stm32SpiBus.h` | [hal](hal.md#stm32spibus) |
| `Stm32UartPort` | class | `hal/stm32/Stm32UartPort.h` | [hal](hal.md#stm32uartport) |
| `TakeoffSequencer` | class | `autopilot/feedback/TakeoffSequencer.h` | [feedback](feedback.md#takeoffsequencer) |
| `ThrottleManager` | class | `control/ThrottleManager.h` | [control](control.md#throttlemanager) |
| `UbloxM10_Gps` | class | `sensors/gps/UbloxM10_Gps.h` | [sensors](sensors.md#ubloxm10_gps) |
| `WebDashboardPage` | namespace | `telemetry/WebDashboardPage.h` | [telemetry](telemetry.md#webdashboardpage) |
| `WebDebugServer` | class | `telemetry/WebDebugServer.h` | [telemetry](telemetry.md#webdebugserver) |

## Условные обозначения

- **Единицы:** мкс — микросекунды PWM или отклонения (±500 = полный ход);
  °, °/с, °/с² — углы и их производные в авиационных знаках (крен + правое
  крыло вниз, тангаж + нос вверх, рысканье + нос вправо); g — ускорение
  в долях g; оси самолёта: X к носу, Y влево, Z вверх.
- **Время:** `millis()`/`micros()` — `uint32_t`; все разности времени
  считаются беззнаковым вычитанием и корректно переживают переполнение
  (≈49.7 суток для `millis()`, ≈71.6 минуты для `micros()`).
- **«Заготовка»** — код есть и проверен тестами, но в прошивку не подключён.
