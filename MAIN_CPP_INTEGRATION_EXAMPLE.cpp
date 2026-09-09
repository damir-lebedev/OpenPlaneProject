// ============================================================
// 🛩️ MAIN.CPP INTEGRATION EXAMPLE (WITH AUTOPILOT)
//
// This file shows how to integrate:
//   • Autopilot system
//   • IMU sensor (MPU6050)
//   • Barometer sensor (BME280)
//   • Feature Manager (CH7-CH10 mapping)
//   • Web Debug Server (WiFi dashboard)
//
// Copy-paste the relevant sections into your main.cpp
// ============================================================

#include "../include/include.h"
#include "../include/sensors/SensorInterface.h"
#include "../include/sensors/MPU6050_Sensor.h"
#include "../include/sensors/BME280_Sensor.h"
#include "../include/Autopilot.h"
#include "../include/FeatureManager.h"
#include "../include/WebDebugServer.h"

// ============================================================
// GLOBAL COMPONENT DECLARATIONS
// ============================================================

// Existing components (from original main.cpp)
HardwareSerial IBusSerial(1);
IBusReceiver ibusReceiver(IBusSerial);
ControlMixer controlMixer;
ThrottleManager throttleManager;
ArmingManager armingManager;
FlightOutputs flightOutputs;
DebugLogger debugLogger;

// New components (Autopilot system)
MPU6050_Sensor imuSensor(0x68);          // IMU (Gyro + Accel)
BME280_Sensor baroSensor(0x76);          // Barometer (Altitude)
Autopilot autopilot(&imuSensor, &baroSensor);  // Autopilot with sensors
FeatureManager featureManager(&autopilot); // Feature mapping (CH7-CH10)
WebDebugServer webServer(&flightController, &autopilot, &featureManager);

// FlightController WITH Autopilot integration
FlightController flightController(
    ibusReceiver,
    controlMixer,
    throttleManager,
    armingManager,
    flightOutputs,
    &autopilot,           // NEW: Autopilot
    &featureManager       // NEW: Feature Manager
);


// ============================================================
// SETUP() - INITIALIZATION
// ============================================================

void setup()
{
    // ========================================================
    // STEP 1: Initialize Serial for debugging
    // ========================================================
    Serial.begin(115200);
    delay(1000);  // Wait for serial to be ready

    Serial.println("\n\n");
    Serial.println("╔════════════════════════════════════════════════════╗");
    Serial.println("║          🛩️  AEROS-001 FLIGHT CONTROLLER           ║");
    Serial.println("║         OpenPlane Project with Autopilot          ║");
    Serial.println("║               ESP32-C3 SuperMini                  ║");
    Serial.println("╚════════════════════════════════════════════════════╝");
    Serial.println();


    // ========================================================
    // STEP 2: Initialize I2C for sensors
    // ========================================================
    Serial.println("🔌 Initializing I2C bus...");
    Wire.begin(21, 22);  // SDA=GPIO21, SCL=GPIO22
    Wire.setClock(400000);  // 400 kHz
    Serial.println("   ✓ I2C initialized");
    Serial.println();


    // ========================================================
    // STEP 3: Initialize sensors
    // ========================================================
    Serial.println("📊 Initializing sensors...");

    if (!imuSensor.begin())
    {
        Serial.println("   ❌ IMU sensor FAILED - system will work without gyro");
        Serial.println("      (Check I2C connection, address 0x68)");
    }

    if (!baroSensor.begin())
    {
        Serial.println("   ❌ Barometer sensor FAILED - system will work without altitude");
        Serial.println("      (Check I2C connection, address 0x76)");
    }

    Serial.println();


    // ========================================================
    // STEP 4: Calibrate sensors
    // ========================================================
    // IMPORTANT: Keep aircraft level and stationary!
    Serial.println("🔧 CALIBRATING SENSORS (Keep aircraft level and still!)");
    Serial.println("   ⏳ This takes ~3 seconds...");

    if (imuSensor.isAvailable())
    {
        Serial.println("   🔄 Calibrating IMU (gyroscope offsets)...");
        imuSensor.calibrate();
        Serial.println("   ✓ IMU calibration complete");
    }

    delay(500);

    if (baroSensor.isAvailable())
    {
        Serial.println("   🔄 Calibrating Barometer (baseline altitude)...");
        baroSensor.calibrateAltitude();
        Serial.println("   ✓ Barometer calibration complete");
    }

    Serial.println();


    // ========================================================
    // STEP 5: Initialize autopilot
    // ========================================================
    Serial.println("🚀 Initializing Autopilot...");
    if (!autopilot.begin())
    {
        Serial.println("   ⚠️  Autopilot initialization warning");
    }
    Serial.println();


    // ========================================================
    // STEP 6: Initialize feature manager
    // ========================================================
    Serial.println("🎛️  Initializing Feature Manager...");
    if (!featureManager.begin())
    {
        Serial.println("   ⚠️  Feature Manager initialization warning");
    }
    Serial.println();


    // ========================================================
    // STEP 7: Initialize flight controller
    // ========================================================
    Serial.println("🎮 Initializing Flight Controller...");
    flightController.begin();
    Serial.println("   ✓ Flight Controller ready");
    Serial.println();


    // ========================================================
    // STEP 8: Initialize WiFi and web server
    // ========================================================
    Serial.println("🌐 Initializing WiFi & Web Server...");
    if (!webServer.begin(0))  // 0 = AP mode
    {
        Serial.println("   ⚠️  Web Server failed to start");
    }
    Serial.println();


    // ========================================================
    // STEP 9: Print system status
    // ========================================================
    Serial.println("════════════════════════════════════════════════════");
    Serial.println("✅ SYSTEM READY!");
    Serial.println("════════════════════════════════════════════════════");
    Serial.println();
    Serial.println("📋 SYSTEM STATUS:");
    Serial.println("────────────────────────────────────────────────────");

    imuSensor.printStatus();
    baroSensor.printStatus();
    autopilot.printStatus();
    featureManager.printStatus();
    webServer.printStatus();

    Serial.println();
    Serial.println("🔗 To debug via Web Browser:");
    Serial.println("   1. Connect to WiFi: 'OpenPlane-Debug'");
    Serial.println("   2. Password: '12345678'");
    Serial.println("   3. Open: http://192.168.4.1");
    Serial.println();
    Serial.println("📡 RC Arming:");
    Serial.println("   Hold throttle (CH3) at MINIMUM for 2 seconds");
    Serial.println("   Status will be shown in the console");
    Serial.println();
    Serial.println("🚀 Autopilot Modes:");
    Serial.println("   CH7 (>1500µs) → AUTO_TAKEOFF");
    Serial.println("   CH8 (>1500µs) → ALT_HOLD");
    Serial.println("   CH9 (>1500µs) → STABILIZE");
    Serial.println("   CH10(>1500µs) → MANUAL");
    Serial.println();
}


// ============================================================
// LOOP() - MAIN CONTROL LOOP
// ============================================================
// This runs at ~500 Hz (every 2ms)

void loop()
{
    // ========================================================
    // MAIN FLIGHT CONTROL CYCLE
    // ========================================================
    // 1. Update flight controller (orchestrates everything)
    flightController.update();

    // 2. Update feature manager (processes CH7-CH10)
    // (Already called from inside FlightController::update())

    // 3. Update autopilot (sensor fusion, PID loops)
    // (Already called from inside FlightController::update())

    // 4. Handle WiFi requests
    webServer.update();

    // 5. Debug logging (optional, every 500ms)
    debugLogger.update();

    // 6. Keep timing consistent
    delay(2);  // 500 Hz = 2ms per cycle


    // ========================================================
    // DIAGNOSTIC OUTPUT (Every ~2 seconds)
    // ========================================================
    // Uncomment this section for periodic status output

    /*
    static unsigned long lastDiagTime = 0;
    if (millis() - lastDiagTime > 2000)
    {
        lastDiagTime = millis();

        Serial.println("\n════════════════════════════════════════════════════");
        Serial.println("📊 DIAGNOSTIC OUTPUT");
        Serial.println("════════════════════════════════════════════════════");

        // Flight status
        Serial.print("🎮 Status: ");
        Serial.print(flightController.isArmed() ? "ARMED ✓" : "DISARMED");
        Serial.print("  Failsafe: ");
        Serial.println(flightController.isReceiverFailsafe() ? "YES ❌" : "NO ✓");

        // Sensor data
        imuSensor.printStatus();
        baroSensor.printStatus();

        // Autopilot data
        autopilot.printStatus();
        featureManager.printStatus();

        Serial.println();
    }
    */
}


// ============================================================
// OPTIONAL: RC DEBUG FUNCTION
// ============================================================
// Call this from Serial monitor to see RC channel values

void printRCChannels()
{
    const RcChannelState& rc = flightController.getRcState();

    Serial.println("\n📡 RC CHANNEL VALUES (µs):");
    Serial.println("─────────────────────────────────────────────────────");

    for (int i = 1; i <= 10; i++)
    {
        uint16_t value = rc.get(i);
        const char* name = "";

        switch (i)
        {
            case 1: name = "CH1 (Aileron)"; break;
            case 2: name = "CH2 (Elevator)"; break;
            case 3: name = "CH3 (Throttle)"; break;
            case 4: name = "CH4 (Rudder)"; break;
            case 5: name = "CH5 (Flaps)"; break;
            case 6: name = "CH6 (Boost)"; break;
            case 7: name = "CH7 (AutoTakeoff)"; break;
            case 8: name = "CH8 (AltHold)"; break;
            case 9: name = "CH9 (Stabilize)"; break;
            case 10: name = "CH10 (Manual)"; break;
        }

        Serial.print("  ");
        Serial.print(name);
        Serial.print(": ");
        Serial.print(value);
        Serial.println(" µs");
    }

    Serial.println("─────────────────────────────────────────────────────\n");
}


// ============================================================
// OPTIONAL: CHANGE AUTOPILOT MODE MANUALLY
// ============================================================

void setAutopilotMode(AutopilotMode mode)
{
    autopilot.setMode(mode);
    const char* modeNames[] = {"MANUAL", "STABILIZE", "AUTO_TAKEOFF", "ALT_HOLD"};
    Serial.print("🚀 Autopilot mode changed to: ");
    Serial.println(modeNames[mode]);
}


// ============================================================
// OPTIONAL: PRINT SENSOR DIAGNOSTICS
// ============================================================

void printSensorDiagnostics()
{
    Serial.println("\n📊 SENSOR DIAGNOSTICS:");
    Serial.println("─────────────────────────────────────────────────────");

    imuSensor.printStatus();
    Serial.println();
    baroSensor.printStatus();
    Serial.println();
    autopilot.printStatus();

    Serial.println("─────────────────────────────────────────────────────\n");
}


// ============================================================
// OPTIONAL: TUNE PID GAINS
// ============================================================

void tunePIDGains(float kpRoll, float kiRoll, float kdRoll,
                  float kpPitch, float kiPitch, float kdPitch)
{
    Serial.print("🔧 Tuning PID gains...");
    autopilot.setPIDGains(kpRoll, kiRoll, kdRoll, kpPitch, kiPitch, kdPitch);
    Serial.println(" Done!");

    Serial.print("   Roll:  Kp=");
    Serial.print(kpRoll);
    Serial.print(" Ki=");
    Serial.print(kiRoll);
    Serial.print(" Kd=");
    Serial.println(kdRoll);

    Serial.print("   Pitch: Kp=");
    Serial.print(kpPitch);
    Serial.print(" Ki=");
    Serial.print(kiPitch);
    Serial.print(" Kd=");
    Serial.println(kdPitch);
}
