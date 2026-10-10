# Klassreferens

> 🌐 Den här sidan är en översättning av [det ryska originalet](../../../reference/README.md). Om översättningen och originalet skiljer sig åt gäller originalet. Firmwaren skriver ut sina konsolmeddelanden på ryska, så de citeras oförändrade. Översättningen är gjord av AI och har inte granskats av personer med svenska som modersmål. Rapportera fel till [Damir Lebedev](https://github.com/damir-lebedev) eller i [ärendehanteraren](https://github.com/damir-lebedev/OpenPlaneProject/issues).

En fullständig lista över firmwarens klasser, strukturer, uppräkningar och namnrymder,
grupperade efter lager. Helhetsbilden (lager, dataflöden, tillståndsmaskiner) finns i
[`../ARCHITECTURE.md`](../ARCHITECTURE.md); praktiska recept finns i
[`../DEVELOPER_GUIDE.md`](../DEVELOPER_GUIDE.md).

Alla sökvägar är relativa till `include/`, utom `src/`. Alla klasser är header-only (varianten med `.h/.cpp` finns i grenen `feature/split-headers`).

| Sida | Lager | Vad som finns där |
|---|---|---|
| [config.md](config.md) | KONFIGURATION | `Config`, `Channels`, `Controls` |
| [hal.md](hal.md) | HAL | `IBoard`, `ServoChannel`, `II2CBus`, `ISpiBus`, `IUartPort`, `IServoOutput`, `IRegisterDevice`, `I2cRegisterDevice`, `SpiRegisterDevice`, `Rtos`, `Esp32Board`, `Esp32I2CBus`, `Esp32SpiBus`, `Esp32UartPort`, `Esp32ServoOutput`; STM32H743: `Stm32Board`, `Stm32I2CBus`, `Stm32SpiBus`, `Stm32UartPort`, `Stm32ServoOutput`, `Stm32FlashStorage`, `compat/Preferences` |
| [storage.md](storage.md) | LAGRING | `IFlashStorage`, `KeyValueStore`, `KvPreferences` |
| [rc.md](rc.md) | RC | `RcChannelState`, `RcInput`, `IBusReceiver` |
| [control.md](control.md) | STYRNING / KOORDINERING | `ControlCommand`, `FlightOutputState`, `FlapsController`, `ControlMixer`, `ThrottleManager`, `ArmingManager`, `FlightOutputs`, `Beeper`, `FlightController` |
| [autopilot.md](autopilot.md) | AUTOPILOT | `AutopilotMode`, `Feature`, `Knob`, `PilotInputs`, `Binding`/`Bind`/`BindingCheck`, `PilotSwitches`, `Autopilot`, `NavStatus`, `Geo`/`Guidance`, `AltitudeSpeedController`, `LaunchController`, `SoaringController`, `AutoTrim`, `PidController` |
| [feedback.md](feedback.md) | AUTOPILOT / återkoppling | `FeedbackConfig`, `FeedbackMath`, `FlightSnapshot`, `FeedbackOutput`, `PhaseTargets`, `SpeedEstimator`, `AirborneDetector`, `ControlEffectivenessEstimator`, `AxisModel`, `AdaptiveRateController`, `StallGuard`, `TakeoffSequencer`, `LandingSequencer`, `FeedbackSupervisor` |
| [sensors.md](sensors.md) | SENSORER | gränssnitt och data, `SensorMounting`, `SensorSelection`, `ImuOrientation`, `AttitudeEstimator`, `ImuSensorBase`, `MPU6050_Sensor`, `ICM42688_Sensor`, `LSM6DSV_Sensor`, `ICM45686_Sensor`, `BarometerBase`, `BMP388_Sensor`, `BME280_Sensor`, `SPL06_Sensor`, `BMP581_Sensor`, `MagnetometerBase`, `QMC5883P_Sensor`, `QMC5883L_Sensor`, `QMC6309_Sensor`, `UbloxM10_Gps`, `AirspeedSensor`, `PitotDualBaroAirspeed` |
| [telemetry.md](telemetry.md) | TELEMETRI | `LoopStats`, `LogChannel`, `LogMode`, `LogSettings`, `DebugLogger`, `DebugConsole`, `WebDashboardPage`, `WebDebugServer`, `OledDisplay`, `Mavlink` (kodek), `MavlinkModes`, `MavlinkTelemetry` |
| [application.md](application.md) | APPLIKATION | `src/main.cpp` (ESP32) och `src/stm32/main.cpp` (STM32H743): globala objekt, `setup()`, slingan och uppgifterna |

## Alfabetiskt register

| Entitet | Typ | Fil | Sida |
|---|---|---|---|
| `AdaptiveRateController` | klass | `autopilot/feedback/AdaptiveRateController.h` | [feedback](feedback.md#adaptiveratecontroller) |
| `AirborneDetector` | klass | `autopilot/feedback/AirborneDetector.h` | [feedback](feedback.md#airbornedetector) |
| `AirspeedData`, `AirspeedSensor` | struktur, gränssnitt | `sensors/airspeed/AirspeedSensor.h` | [sensors](sensors.md#airspeedsensor) |
| `AltitudeSpeedController` | klass | `autopilot/AltitudeSpeedController.h` | [autopilot](autopilot.md#altitudespeedcontroller) |
| `ArmingManager` | klass | `control/ArmingManager.h` | [control](control.md#armingmanager) |
| `AttitudeEstimator` | klass | `sensors/imu/AttitudeEstimator.h` | [sensors](sensors.md#attitudeestimator) |
| `Autopilot` | klass | `autopilot/Autopilot.h` | [autopilot](autopilot.md#autopilot) |
| `AutopilotMode` | uppräkning | `autopilot/AutopilotTypes.h` | [autopilot](autopilot.md#autopilotmode-feature-knob) |
| `AutoTrim` | klass | `autopilot/AutoTrim.h` | [autopilot](autopilot.md#autotrim) |
| `AxisModel` | struktur | `autopilot/feedback/AdaptiveRateController.h` | [feedback](feedback.md#axismodel) |
| `BarometerBase` | abstrakt klass | `sensors/baro/BarometerBase.h` | [sensors](sensors.md#barometerbase) |
| `BarometerData`, `BarometerSensor` | struktur, gränssnitt | `sensors/SensorInterface.h` | [sensors](sensors.md#gränssnitt-och-datastrukturer) |
| `Beeper` | klass | `control/Beeper.h` | [control](control.md#beeper) |
| `Binding`, `Bind`, `BindingCheck` | struktur, namnrymd | `autopilot/ControlBinding.h` | [autopilot](autopilot.md#binding-bind-bindingcheck) |
| `BME280_Sensor` | klass | `sensors/baro/BME280_Sensor.h` | [sensors](sensors.md#bme280_sensor) |
| `BMP388_Sensor` | klass | `sensors/baro/BMP388_Sensor.h` | [sensors](sensors.md#bmp388_sensor) |
| `BMP581_Sensor` | klass | `sensors/baro/BMP581_Sensor.h` | [sensors](sensors.md#bmp581_sensor) |
| `Channels` | namnrymd | `config/Channels.h` | [config](config.md#namnrymd-channels) |
| `Config` | namnrymd | `config/Config.h` | [config](config.md#namnrymd-config) |
| `ControlCommand` | struktur | `control/ControlCommand.h` | [control](control.md#controlcommand) |
| `ControlEffectivenessEstimator` | klass | `autopilot/feedback/ControlEffectivenessEstimator.h` | [feedback](feedback.md#controleffectivenessestimator) |
| `ControlMixer` | klass | `control/ControlMixer.h` | [control](control.md#controlmixer) |
| `Controls` | namnrymd | `config/Controls.h` | [config](config.md#namnrymd-controls) |
| `DebugConsole` | klass | `telemetry/DebugConsole.h` | [telemetry](telemetry.md#debugconsole) |
| `DebugLogger` | klass | `telemetry/DebugLogger.h` | [telemetry](telemetry.md#debuglogger) |
| `Esp32Board` | klass | `hal/esp32/Esp32Board.h` | [hal](hal.md#esp32board) |
| `Esp32I2CBus` | klass | `hal/esp32/Esp32I2CBus.h` | [hal](hal.md#esp32i2cbus) |
| `Esp32ServoOutput` | klass | `hal/esp32/Esp32ServoOutput.h` | [hal](hal.md#esp32servooutput) |
| `Esp32SpiBus` | klass | `hal/esp32/Esp32SpiBus.h` | [hal](hal.md#esp32spibus) |
| `Esp32UartPort` | klass | `hal/esp32/Esp32UartPort.h` | [hal](hal.md#esp32uartport) |
| `Feature`, `Knob`, `PilotInputs` | uppräkning, struktur | `autopilot/AutopilotTypes.h` | [autopilot](autopilot.md#autopilotmode-feature-knob) |
| `FeedbackConfig` | namnrymd | `autopilot/feedback/FeedbackConfig.h` | [feedback](feedback.md#namnrymd-feedbackconfig) |
| `FeedbackMath` | namnrymd | `autopilot/feedback/FeedbackMath.h` | [feedback](feedback.md#namnrymd-feedbackmath) |
| `FeedbackOutput` | struktur | `autopilot/feedback/FeedbackOutput.h` | [feedback](feedback.md#feedbackoutput) |
| `FeedbackSupervisor` | klass | `autopilot/feedback/FeedbackSupervisor.h` | [feedback](feedback.md#feedbacksupervisor) |
| `FlapsController` | klass | `control/FlapsController.h` | [control](control.md#flapscontroller) |
| `FlightController` | klass | `control/FlightController.h` | [control](control.md#flightcontroller) |
| `FlightOutputs`, `FlightOutputs::OutputInfo` | klass, struktur | `control/FlightOutputs.h` | [control](control.md#flightoutputs) |
| `FlightOutputState` | struktur | `control/FlightOutputState.h` | [control](control.md#flightoutputstate) |
| `FlightSnapshot` | struktur | `autopilot/feedback/FlightSnapshot.h` | [feedback](feedback.md#flightsnapshot) |
| `Geo`, `Guidance`, `GeoPoint` | namnrymd, struktur | `autopilot/Navigation.h` | [autopilot](autopilot.md#geo-guidance-geopoint) |
| `GpsData`, `GpsSensor` | struktur, gränssnitt | `sensors/SensorInterface.h` | [sensors](sensors.md#gränssnitt-och-datastrukturer) |
| `I2cRegisterDevice` | klass | `hal/RegisterDevice.h` | [hal](hal.md#i2cregisterdevice) |
| `IBoard` | gränssnitt | `hal/IBoard.h` | [hal](hal.md#iboard) |
| `IBusReceiver` | klass | `rc/IBusReceiver.h` | [rc](rc.md#ibusreceiver) |
| `ICM42688_Sensor` | klass | `sensors/imu/ICM42688_Sensor.h` | [sensors](sensors.md#icm42688_sensor) |
| `ICM45686_Sensor` | klass | `sensors/imu/ICM45686_Sensor.h` | [sensors](sensors.md#icm45686_sensor) |
| `IFlashStorage` | gränssnitt | `storage/KeyValueStore.h` | [storage](storage.md#iflashstorage) |
| `II2CBus` | gränssnitt | `hal/II2CBus.h` | [hal](hal.md#ii2cbus) |
| `ImuData`, `ImuSensor` | struktur, gränssnitt | `sensors/SensorInterface.h` | [sensors](sensors.md#gränssnitt-och-datastrukturer) |
| `ImuOrientation` | klass | `sensors/imu/ImuOrientation.h` | [sensors](sensors.md#imuorientation) |
| `ImuSensorBase` | abstrakt klass | `sensors/imu/ImuSensorBase.h` | [sensors](sensors.md#imusensorbase) |
| `IRegisterDevice` | gränssnitt | `hal/RegisterDevice.h` | [hal](hal.md#iregisterdevice) |
| `IServoOutput` | gränssnitt | `hal/IServoOutput.h` | [hal](hal.md#iservooutput) |
| `ISpiBus` | gränssnitt | `hal/ISpiBus.h` | [hal](hal.md#ispibus) |
| `IUartPort` | gränssnitt | `hal/IUartPort.h` | [hal](hal.md#iuartport) |
| `KeyValueStore` | klass | `storage/KeyValueStore.h` | [storage](storage.md#keyvaluestore) |
| `KvPreferences` | klass | `storage/KvPreferences.h` | [storage](storage.md#kvpreferences) |
| `LandingSequencer` | klass | `autopilot/feedback/LandingSequencer.h` | [feedback](feedback.md#landingsequencer) |
| `LaunchController` | klass | `autopilot/LaunchController.h` | [autopilot](autopilot.md#launchcontroller) |
| `LogChannel`, `LogMode`, `LogChannelInfo` | uppräkning, uppräkning, struktur | `telemetry/LogSettings.h` | [telemetry](telemetry.md#logsettings) |
| `LogSettings` | klass | `telemetry/LogSettings.h` | [telemetry](telemetry.md#logsettings) |
| `LoopStats` | struktur | `telemetry/LoopStats.h` | [telemetry](telemetry.md#loopstats) |
| `LSM6DSV_Sensor` | klass | `sensors/imu/LSM6DSV_Sensor.h` | [sensors](sensors.md#lsm6dsv_sensor) |
| `MagData`, `MagnetometerSensor` | struktur, gränssnitt | `sensors/SensorInterface.h` | [sensors](sensors.md#gränssnitt-och-datastrukturer) |
| `MagnetometerBase` | abstrakt klass | `sensors/mag/MagnetometerBase.h` | [sensors](sensors.md#magnetometerbase) |
| `Mavlink` | namnrymd | `telemetry/MavlinkCodec.h` | [telemetry](telemetry.md#mavlink-kodek) |
| `MavlinkModes`, `MavlinkTelemetry` | namnrymd, klass | `telemetry/MavlinkTelemetry.h` | [telemetry](telemetry.md#mavlinktelemetry) |
| `MPU6050_Sensor` | klass | `sensors/imu/MPU6050_Sensor.h` | [sensors](sensors.md#mpu6050_sensor) |
| `OledDisplay` | klass | `telemetry/OledDisplay.h` | [telemetry](telemetry.md#oleddisplay) |
| `PhaseTargets` | struktur | `autopilot/feedback/PhaseTargets.h` | [feedback](feedback.md#phasetargets) |
| `PidController` | klass | `autopilot/PidController.h` | [autopilot](autopilot.md#pidcontroller) |
| `PilotSwitches` | klass | `autopilot/PilotSwitches.h` | [autopilot](autopilot.md#pilotswitches) |
| `PitotDualBaroAirspeed` | klass | `sensors/airspeed/PitotDualBaroAirspeed.h` | [sensors](sensors.md#pitotdualbaroairspeed) |
| `QMC5883L_Sensor` | klass | `sensors/mag/QMC5883L_Sensor.h` | [sensors](sensors.md#qmc5883l_sensor) |
| `QMC5883P_Sensor` | klass | `sensors/mag/QMC5883P_Sensor.h` | [sensors](sensors.md#qmc5883p_sensor) |
| `QMC6309_Sensor` | klass | `sensors/mag/QMC6309_Sensor.h` | [sensors](sensors.md#qmc6309_sensor) |
| `RawImuSample` | struktur | `sensors/imu/ImuSensorBase.h` | [sensors](sensors.md#imusensorbase) |
| `RcChannelState` | klass | `rc/RcChannelState.h` | [rc](rc.md#rcchannelstate) |
| `RcInput` | klass (statisk) | `rc/RcInput.h` | [rc](rc.md#rcinput) |
| `Rtos` | namnrymd | `hal/Rtos.h` | [hal](hal.md#rtos) |
| `Sensor` | gränssnitt | `sensors/SensorInterface.h` | [sensors](sensors.md#gränssnitt-och-datastrukturer) |
| `SensorMounting` | namnrymd | `sensors/SensorMounting.h` | [sensors](sensors.md#namnrymd-sensormounting) |
| `SensorSelection.h` | makron | `sensors/SensorSelection.h` | [sensors](sensors.md#sensorselectionh) |
| `ServoChannel` | namnrymd | `hal/IBoard.h` | [hal](hal.md#namnrymd-servochannel) |
| `SoaringController` | klass | `autopilot/SoaringController.h` | [autopilot](autopilot.md#soaringcontroller) |
| `SpeedEstimator` | klass | `autopilot/feedback/SpeedEstimator.h` | [feedback](feedback.md#speedestimator) |
| `SpiRegisterDevice` | klass | `hal/RegisterDevice.h` | [hal](hal.md#spiregisterdevice) |
| `SPL06_Sensor` | klass | `sensors/baro/SPL06_Sensor.h` | [sensors](sensors.md#spl06_sensor) |
| `StallGuard` | klass | `autopilot/feedback/StallGuard.h` | [feedback](feedback.md#stallguard) |
| `Stm32Board` | klass | `hal/stm32/Stm32Board.h` | [hal](hal.md#stm32board) |
| `Stm32FlashStorage` | klass | `hal/stm32/Stm32FlashStorage.h` | [hal](hal.md#stm32flashstorage) |
| `Stm32I2CBus` | klass | `hal/stm32/Stm32I2CBus.h` | [hal](hal.md#stm32i2cbus) |
| `Stm32ServoOutput` | klass | `hal/stm32/Stm32ServoOutput.h` | [hal](hal.md#stm32servooutput) |
| `Stm32SpiBus` | klass | `hal/stm32/Stm32SpiBus.h` | [hal](hal.md#stm32spibus) |
| `Stm32UartPort` | klass | `hal/stm32/Stm32UartPort.h` | [hal](hal.md#stm32uartport) |
| `TakeoffSequencer` | klass | `autopilot/feedback/TakeoffSequencer.h` | [feedback](feedback.md#takeoffsequencer) |
| `ThrottleManager` | klass | `control/ThrottleManager.h` | [control](control.md#throttlemanager) |
| `UbloxM10_Gps` | klass | `sensors/gps/UbloxM10_Gps.h` | [sensors](sensors.md#ubloxm10_gps) |
| `WebDashboardPage` | namnrymd | `telemetry/WebDashboardPage.h` | [telemetry](telemetry.md#webdashboardpage) |
| `WebDebugServer` | klass | `telemetry/WebDebugServer.h` | [telemetry](telemetry.md#webdebugserver) |
## Konventioner

- **Enheter:** µs – PWM-mikrosekunder eller utslag (±500 = fullt utslag); °, °/s, °/s² – vinklar och
  deras derivator med flygtekniska tecken (roll + höger vinge ned, tippning + nosen upp, gir + nosen åt höger);
  g – acceleration i bråkdelar av g; flygplanets axlar: X mot nosen, Y åt vänster, Z uppåt.
- **Tid:** `millis()`/`micros()` – `uint32_t`; alla tidsskillnader beräknas med subtraktion utan
  tecken och klarar korrekt överslag (≈49,7 dygn för `millis()`, ≈71,6 minuter för
  `micros()`).
- **”Förarbete”** – koden finns och täcks av tester, men är inte ansluten till firmwaren.
