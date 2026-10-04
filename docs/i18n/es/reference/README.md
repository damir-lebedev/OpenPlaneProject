# Referencia de clases

> 🌐 Esta página es una traducción del [original en ruso](../../../reference/README.md). Si la traducción y el original difieren, prevalece el original. El firmware muestra los mensajes de la consola en ruso, por lo que se citan tal cual. La traducción la ha hecho una IA y no la han revisado hablantes nativos. Si encuentras errores, escribe a [Damir Lebedev](https://github.com/damir-lebedev) o abre una [incidencia](https://github.com/damir-lebedev/OpenPlaneProject/issues).

Lista completa de las clases, estructuras, enumeraciones y espacios de nombres del firmware,
agrupada por capas. La visión de conjunto (capas, flujos, máquinas de estados) está en
[`../ARCHITECTURE.md`](../ARCHITECTURE.md); las recetas prácticas, en
[`../DEVELOPER_GUIDE.md`](../DEVELOPER_GUIDE.md).

Todas las rutas son relativas a `include/`, salvo `src/`. Todas las clases son solo de cabecera (la variante con `.h/.cpp` está en la rama `feature/split-headers`).

| Página | Capa | Qué contiene |
|---|---|---|
| [config.md](config.md) | CONFIG | `Config`, `Channels`, `Controls` |
| [hal.md](hal.md) | HAL | `IBoard`, `ServoChannel`, `II2CBus`, `ISpiBus`, `IUartPort`, `IServoOutput`, `IRegisterDevice`, `I2cRegisterDevice`, `SpiRegisterDevice`, `Rtos`, `Esp32Board`, `Esp32I2CBus`, `Esp32SpiBus`, `Esp32UartPort`, `Esp32ServoOutput`; STM32H743: `Stm32Board`, `Stm32I2CBus`, `Stm32SpiBus`, `Stm32UartPort`, `Stm32ServoOutput`, `Stm32FlashStorage`, `compat/Preferences` |
| [storage.md](storage.md) | STORAGE | `IFlashStorage`, `KeyValueStore`, `KvPreferences` |
| [rc.md](rc.md) | RC | `RcChannelState`, `RcInput`, `IBusReceiver` |
| [control.md](control.md) | CONTROL / COORDINATION | `ControlCommand`, `FlightOutputState`, `FlapsController`, `ControlMixer`, `ThrottleManager`, `ArmingManager`, `FlightOutputs`, `Beeper`, `FlightController` |
| [autopilot.md](autopilot.md) | AUTOPILOT | `AutopilotMode`, `Feature`, `Knob`, `PilotInputs`, `Binding`/`Bind`/`BindingCheck`, `PilotSwitches`, `Autopilot`, `NavStatus`, `Geo`/`Guidance`, `AltitudeSpeedController`, `LaunchController`, `SoaringController`, `AutoTrim`, `PidController` |
| [feedback.md](feedback.md) | AUTOPILOT / feedback | `FeedbackConfig`, `FeedbackMath`, `FlightSnapshot`, `FeedbackOutput`, `PhaseTargets`, `SpeedEstimator`, `AirborneDetector`, `ControlEffectivenessEstimator`, `AxisModel`, `AdaptiveRateController`, `StallGuard`, `TakeoffSequencer`, `LandingSequencer`, `FeedbackSupervisor` |
| [sensors.md](sensors.md) | SENSORS | interfaces y datos, `SensorMounting`, `SensorSelection`, `ImuOrientation`, `AttitudeEstimator`, `ImuSensorBase`, `MPU6050_Sensor`, `ICM42688_Sensor`, `LSM6DSV_Sensor`, `ICM45686_Sensor`, `BarometerBase`, `BMP388_Sensor`, `BME280_Sensor`, `SPL06_Sensor`, `BMP581_Sensor`, `MagnetometerBase`, `QMC5883P_Sensor`, `QMC5883L_Sensor`, `QMC6309_Sensor`, `UbloxM10_Gps`, `AirspeedSensor`, `PitotDualBaroAirspeed` |
| [telemetry.md](telemetry.md) | TELEMETRY | `LoopStats`, `LogChannel`, `LogMode`, `LogSettings`, `DebugLogger`, `DebugConsole`, `WebDashboardPage`, `WebDebugServer`, `OledDisplay`, `Mavlink` (códec), `MavlinkModes`, `MavlinkTelemetry` |
| [application.md](application.md) | APPLICATION | `src/main.cpp` (ESP32) y `src/stm32/main.cpp` (STM32H743): objetos globales, `setup()`, el bucle y las tareas |

## Índice alfabético

| Entidad | Tipo | Archivo | Página |
|---|---|---|---|
| `AdaptiveRateController` | class | `autopilot/feedback/AdaptiveRateController.h` | [feedback](feedback.md#adaptiveratecontroller) |
| `AirborneDetector` | class | `autopilot/feedback/AirborneDetector.h` | [feedback](feedback.md#airbornedetector) |
| `AirspeedData`, `AirspeedSensor` | struct, interface | `sensors/airspeed/AirspeedSensor.h` | [sensors](sensors.md#airspeedsensor) |
| `AltitudeSpeedController` | class | `autopilot/AltitudeSpeedController.h` | [autopilot](autopilot.md#altitudespeedcontroller) |
| `ArmingManager` | class | `control/ArmingManager.h` | [control](control.md#armingmanager) |
| `AttitudeEstimator` | class | `sensors/imu/AttitudeEstimator.h` | [sensors](sensors.md#attitudeestimator) |
| `Autopilot` | class | `autopilot/Autopilot.h` | [autopilot](autopilot.md#autopilot) |
| `AutopilotMode` | enum | `autopilot/AutopilotTypes.h` | [autopilot](autopilot.md#autopilotmode-feature-knob) |
| `AutoTrim` | class | `autopilot/AutoTrim.h` | [autopilot](autopilot.md#autotrim) |
| `AxisModel` | struct | `autopilot/feedback/AdaptiveRateController.h` | [feedback](feedback.md#axismodel) |
| `BarometerBase` | abstract class | `sensors/baro/BarometerBase.h` | [sensors](sensors.md#barometerbase) |
| `BarometerData`, `BarometerSensor` | struct, interface | `sensors/SensorInterface.h` | [sensors](sensors.md#interfaces-y-estructuras-de-datos) |
| `Beeper` | class | `control/Beeper.h` | [control](control.md#beeper) |
| `Binding`, `Bind`, `BindingCheck` | struct, namespace | `autopilot/ControlBinding.h` | [autopilot](autopilot.md#binding-bind-bindingcheck) |
| `BME280_Sensor` | class | `sensors/baro/BME280_Sensor.h` | [sensors](sensors.md#bme280_sensor) |
| `BMP388_Sensor` | class | `sensors/baro/BMP388_Sensor.h` | [sensors](sensors.md#bmp388_sensor) |
| `BMP581_Sensor` | class | `sensors/baro/BMP581_Sensor.h` | [sensors](sensors.md#bmp581_sensor) |
| `Channels` | namespace | `config/Channels.h` | [config](config.md#namespace-channels) |
| `Config` | namespace | `config/Config.h` | [config](config.md#namespace-config) |
| `ControlCommand` | struct | `control/ControlCommand.h` | [control](control.md#controlcommand) |
| `ControlEffectivenessEstimator` | class | `autopilot/feedback/ControlEffectivenessEstimator.h` | [feedback](feedback.md#controleffectivenessestimator) |
| `ControlMixer` | class | `control/ControlMixer.h` | [control](control.md#controlmixer) |
| `Controls` | namespace | `config/Controls.h` | [config](config.md#namespace-controls) |
| `DebugConsole` | class | `telemetry/DebugConsole.h` | [telemetry](telemetry.md#debugconsole) |
| `DebugLogger` | class | `telemetry/DebugLogger.h` | [telemetry](telemetry.md#debuglogger) |
| `Esp32Board` | class | `hal/esp32/Esp32Board.h` | [hal](hal.md#esp32board) |
| `Esp32I2CBus` | class | `hal/esp32/Esp32I2CBus.h` | [hal](hal.md#esp32i2cbus) |
| `Esp32ServoOutput` | class | `hal/esp32/Esp32ServoOutput.h` | [hal](hal.md#esp32servooutput) |
| `Esp32SpiBus` | class | `hal/esp32/Esp32SpiBus.h` | [hal](hal.md#esp32spibus) |
| `Esp32UartPort` | class | `hal/esp32/Esp32UartPort.h` | [hal](hal.md#esp32uartport) |
| `Feature`, `Knob`, `PilotInputs` | enum, struct | `autopilot/AutopilotTypes.h` | [autopilot](autopilot.md#autopilotmode-feature-knob) |
| `FeedbackConfig` | namespace | `autopilot/feedback/FeedbackConfig.h` | [feedback](feedback.md#namespace-feedbackconfig) |
| `FeedbackMath` | namespace | `autopilot/feedback/FeedbackMath.h` | [feedback](feedback.md#namespace-feedbackmath) |
| `FeedbackOutput` | struct | `autopilot/feedback/FeedbackOutput.h` | [feedback](feedback.md#feedbackoutput) |
| `FeedbackSupervisor` | class | `autopilot/feedback/FeedbackSupervisor.h` | [feedback](feedback.md#feedbacksupervisor) |
| `FlapsController` | class | `control/FlapsController.h` | [control](control.md#flapscontroller) |
| `FlightController` | class | `control/FlightController.h` | [control](control.md#flightcontroller) |
| `FlightOutputs`, `FlightOutputs::OutputInfo` | class, struct | `control/FlightOutputs.h` | [control](control.md#flightoutputs) |
| `FlightOutputState` | struct | `control/FlightOutputState.h` | [control](control.md#flightoutputstate) |
| `FlightSnapshot` | struct | `autopilot/feedback/FlightSnapshot.h` | [feedback](feedback.md#flightsnapshot) |
| `Geo`, `Guidance`, `GeoPoint` | namespace, struct | `autopilot/Navigation.h` | [autopilot](autopilot.md#geo-guidance-geopoint) |
| `GpsData`, `GpsSensor` | struct, interface | `sensors/SensorInterface.h` | [sensors](sensors.md#interfaces-y-estructuras-de-datos) |
| `I2cRegisterDevice` | class | `hal/RegisterDevice.h` | [hal](hal.md#i2cregisterdevice) |
| `IBoard` | interface | `hal/IBoard.h` | [hal](hal.md#iboard) |
| `IBusReceiver` | class | `rc/IBusReceiver.h` | [rc](rc.md#ibusreceiver) |
| `ICM42688_Sensor` | class | `sensors/imu/ICM42688_Sensor.h` | [sensors](sensors.md#icm42688_sensor) |
| `ICM45686_Sensor` | class | `sensors/imu/ICM45686_Sensor.h` | [sensors](sensors.md#icm45686_sensor) |
| `IFlashStorage` | interface | `storage/KeyValueStore.h` | [storage](storage.md#iflashstorage) |
| `II2CBus` | interface | `hal/II2CBus.h` | [hal](hal.md#ii2cbus) |
| `ImuData`, `ImuSensor` | struct, interface | `sensors/SensorInterface.h` | [sensors](sensors.md#interfaces-y-estructuras-de-datos) |
| `ImuOrientation` | class | `sensors/imu/ImuOrientation.h` | [sensors](sensors.md#imuorientation) |
| `ImuSensorBase` | abstract class | `sensors/imu/ImuSensorBase.h` | [sensors](sensors.md#imusensorbase) |
| `IRegisterDevice` | interface | `hal/RegisterDevice.h` | [hal](hal.md#iregisterdevice) |
| `IServoOutput` | interface | `hal/IServoOutput.h` | [hal](hal.md#iservooutput) |
| `ISpiBus` | interface | `hal/ISpiBus.h` | [hal](hal.md#ispibus) |
| `IUartPort` | interface | `hal/IUartPort.h` | [hal](hal.md#iuartport) |
| `KeyValueStore` | class | `storage/KeyValueStore.h` | [storage](storage.md#keyvaluestore) |
| `KvPreferences` | class | `storage/KvPreferences.h` | [storage](storage.md#kvpreferences) |
| `LandingSequencer` | class | `autopilot/feedback/LandingSequencer.h` | [feedback](feedback.md#landingsequencer) |
| `LaunchController` | class | `autopilot/LaunchController.h` | [autopilot](autopilot.md#launchcontroller) |
| `LogChannel`, `LogMode`, `LogChannelInfo` | enum, enum, struct | `telemetry/LogSettings.h` | [telemetry](telemetry.md#logsettings) |
| `LogSettings` | class | `telemetry/LogSettings.h` | [telemetry](telemetry.md#logsettings) |
| `LoopStats` | struct | `telemetry/LoopStats.h` | [telemetry](telemetry.md#loopstats) |
| `LSM6DSV_Sensor` | class | `sensors/imu/LSM6DSV_Sensor.h` | [sensors](sensors.md#lsm6dsv_sensor) |
| `MagData`, `MagnetometerSensor` | struct, interface | `sensors/SensorInterface.h` | [sensors](sensors.md#interfaces-y-estructuras-de-datos) |
| `MagnetometerBase` | abstract class | `sensors/mag/MagnetometerBase.h` | [sensors](sensors.md#magnetometerbase) |
| `Mavlink` | namespace | `telemetry/MavlinkCodec.h` | [telemetry](telemetry.md#mavlink-códec) |
| `MavlinkModes`, `MavlinkTelemetry` | namespace, class | `telemetry/MavlinkTelemetry.h` | [telemetry](telemetry.md#mavlinktelemetry) |
| `MPU6050_Sensor` | class | `sensors/imu/MPU6050_Sensor.h` | [sensors](sensors.md#mpu6050_sensor) |
| `OledDisplay` | class | `telemetry/OledDisplay.h` | [telemetry](telemetry.md#oleddisplay) |
| `PhaseTargets` | struct | `autopilot/feedback/PhaseTargets.h` | [feedback](feedback.md#phasetargets) |
| `PidController` | class | `autopilot/PidController.h` | [autopilot](autopilot.md#pidcontroller) |
| `PilotSwitches` | class | `autopilot/PilotSwitches.h` | [autopilot](autopilot.md#pilotswitches) |
| `PitotDualBaroAirspeed` | class | `sensors/airspeed/PitotDualBaroAirspeed.h` | [sensors](sensors.md#pitotdualbaroairspeed) |
| `QMC5883L_Sensor` | class | `sensors/mag/QMC5883L_Sensor.h` | [sensors](sensors.md#qmc5883l_sensor) |
| `QMC5883P_Sensor` | class | `sensors/mag/QMC5883P_Sensor.h` | [sensors](sensors.md#qmc5883p_sensor) |
| `QMC6309_Sensor` | class | `sensors/mag/QMC6309_Sensor.h` | [sensors](sensors.md#qmc6309_sensor) |
| `RawImuSample` | struct | `sensors/imu/ImuSensorBase.h` | [sensors](sensors.md#imusensorbase) |
| `RcChannelState` | class | `rc/RcChannelState.h` | [rc](rc.md#rcchannelstate) |
| `RcInput` | class (static) | `rc/RcInput.h` | [rc](rc.md#rcinput) |
| `Rtos` | namespace | `hal/Rtos.h` | [hal](hal.md#rtos) |
| `Sensor` | interface | `sensors/SensorInterface.h` | [sensors](sensors.md#interfaces-y-estructuras-de-datos) |
| `SensorMounting` | namespace | `sensors/SensorMounting.h` | [sensors](sensors.md#namespace-sensormounting) |
| `SensorSelection.h` | macros | `sensors/SensorSelection.h` | [sensors](sensors.md#sensorselectionh) |
| `ServoChannel` | namespace | `hal/IBoard.h` | [hal](hal.md#namespace-servochannel) |
| `SoaringController` | class | `autopilot/SoaringController.h` | [autopilot](autopilot.md#soaringcontroller) |
| `SpeedEstimator` | class | `autopilot/feedback/SpeedEstimator.h` | [feedback](feedback.md#speedestimator) |
| `SpiRegisterDevice` | class | `hal/RegisterDevice.h` | [hal](hal.md#spiregisterdevice) |
| `SPL06_Sensor` | class | `sensors/baro/SPL06_Sensor.h` | [sensors](sensors.md#spl06_sensor) |
| `StallGuard` | class | `autopilot/feedback/StallGuard.h` | [feedback](feedback.md#stallguard) |
| `Stm32Board` | class | `hal/stm32/Stm32Board.h` | [hal](hal.md#stm32board) |
| `Stm32FlashStorage` | class | `hal/stm32/Stm32FlashStorage.h` | [hal](hal.md#stm32flashstorage) |
| `Stm32I2CBus` | class | `hal/stm32/Stm32I2CBus.h` | [hal](hal.md#stm32i2cbus) |
| `Stm32ServoOutput` | class | `hal/stm32/Stm32ServoOutput.h` | [hal](hal.md#stm32servooutput) |
| `Stm32SpiBus` | class | `hal/stm32/Stm32SpiBus.h` | [hal](hal.md#stm32spibus) |
| `Stm32UartPort` | class | `hal/stm32/Stm32UartPort.h` | [hal](hal.md#stm32uartport) |
| `TakeoffSequencer` | class | `autopilot/feedback/TakeoffSequencer.h` | [feedback](feedback.md#takeoffsequencer) |
| `ThrottleManager` | class | `control/ThrottleManager.h` | [control](control.md#throttlemanager) |
| `UbloxM10_Gps` | class | `sensors/gps/UbloxM10_Gps.h` | [sensors](sensors.md#ubloxm10_gps) |
| `WebDashboardPage` | namespace | `telemetry/WebDashboardPage.h` | [telemetry](telemetry.md#webdashboardpage) |
| `WebDebugServer` | class | `telemetry/WebDebugServer.h` | [telemetry](telemetry.md#webdebugserver) |

## Convenciones

- **Unidades:** µs: microsegundos de PWM o de deflexión (±500 = recorrido completo); °, °/s, °/s²:
  ángulos y sus derivadas con signos aeronáuticos (alabeo + ala derecha abajo, cabeceo + morro arriba,
  guiñada + morro a la derecha); g: aceleración en fracciones de g; ejes del avión: X hacia el morro,
  Y a la izquierda, Z hacia arriba.
- **Tiempo:** `millis()`/`micros()` son `uint32_t`; todas las diferencias de tiempo se calculan con resta
  sin signo y soportan correctamente el desbordamiento (≈49.7 días para `millis()`, ≈71.6 minutos para
  `micros()`).
- **«Base preparada»:** el código existe y está verificado con pruebas, pero no está conectado al firmware.
