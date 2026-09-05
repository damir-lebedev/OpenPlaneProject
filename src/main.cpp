#include <Arduino.h>

#include <ESP32Servo.h>
// ------------------------------------------------------------
// THROTTLE LIMIT BY CHANNEL 6
// ------------------------------------------------------------
// Канал 6 = 1000 → 100 % мощности
// Канал 6 = 2000 → максимум 40 % мощности (растянуто на весь стик)
constexpr uint16_t THROTTLE_LIMIT_MIN_PERCENT = 40;   // при CH6 = 2000
// ============================================================

// AEROS-001 FLIGHT CONTROLLER

// ESP32-C3 SuperMini

// GPIO 4 -> Aileron R

// GPIO 5 -> Aileron L

// GPIO 6 -> Elevator

// GPIO 7 -> ESC

// GPIO 8 -> PPM input from FS-i6 receiver

//

// PPM channels assumed:

// CH1 = Aileron

// CH2 = Elevator

// CH3 = Throttle

// CH4 = Rudder

// ============================================================

// ------------------------------------------------------------
// PINOUT
// ------------------------------------------------------------

constexpr uint8_t PIN_AILERON_LEFT  = 5;
constexpr uint8_t PIN_AILERON_RIGHT = 4;

constexpr uint8_t PIN_ELEVATOR = 6;

constexpr uint8_t PIN_ESC = 7;

constexpr uint8_t PIN_IBUS = 8;

// ------------------------------------------------------------
// IBUS CONFIGURATION
// ------------------------------------------------------------

constexpr uint8_t IBUS_CHANNELS = 10;

constexpr uint8_t IBUS_FRAME_LENGTH = 32;

constexpr uint8_t IBUS_HEADER_0 = 0x20;
constexpr uint8_t IBUS_HEADER_1 = 0x40;

constexpr uint32_t IBUS_BAUDRATE = 115200;

// Receiver considered lost after this time.

constexpr uint32_t RX_TIMEOUT_US = 100000; // 100 ms
// ------------------------------------------------------------

// SERVO / ESC PULSE LIMITS

// ------------------------------------------------------------

constexpr uint16_t PWM_MIN = 1000;

constexpr uint16_t PWM_CENTER = 1500;

constexpr uint16_t PWM_MAX = 2000;

// ------------------------------------------------------------

// CONTROL LIMITS

// ------------------------------------------------------------

// Maximum control surface deflection.

// Start conservative.

constexpr int16_t AILERON_MAX_US = 1500;

constexpr int16_t ELEVATOR_MAX_US = 1000;

// ------------------------------------------------------------

// THROTTLE SAFETY

// ------------------------------------------------------------

// Throttle below this value is considered LOW.

constexpr uint16_t THROTTLE_LOW_US = 1050;

// Number of milliseconds throttle must stay low before arming

// becomes possible.

constexpr uint32_t ARM_LOW_TIME_MS = 1500;

// ------------------------------------------------------------

// FAILSAFE

// ------------------------------------------------------------

constexpr uint16_t FAILSAFE_AILERON = 1500;

constexpr uint16_t FAILSAFE_ELEVATOR = 1500;

constexpr uint16_t FAILSAFE_THROTTLE = 1000;

// ------------------------------------------------------------

// DEBUG

// ------------------------------------------------------------

constexpr uint32_t DEBUG_INTERVAL_MS = 100;


// ============================================================
// GLOBAL OBJECTS
// ============================================================

Servo servoAileronLeft;
Servo servoAileronRight;

Servo servoElevator;

Servo esc;

// ============================================================
// IBUS DATA
// ============================================================

uint16_t ibusChannels[IBUS_CHANNELS] = {
    1500,
    1500,
    1000,
    1500,
    1500,
    1500,
    1500,
    1500,
    1500,
    1500
};

uint8_t ibusFrame[IBUS_FRAME_LENGTH];

uint8_t ibusFrameIndex = 0;

uint32_t ibusLastFrame = 0;

bool ibusFrameReady = false;

// ============================================================

// RECEIVER STATUS

// ============================================================

bool receiverFailsafe = true;

bool armed = false;

uint32_t throttleLowSince = 0;


// ============================================================
// IBUS RECEIVER
// ============================================================

HardwareSerial IBusSerial(1);

void readIBus()
{
    while (IBusSerial.available())
    {
        uint8_t byte = IBusSerial.read();

        // Looking for frame start
        if (ibusFrameIndex == 0)
        {
            if (byte != IBUS_HEADER_0)
            {
                continue;
            }

            ibusFrame[ibusFrameIndex++] = byte;
            continue;
        }

        if (ibusFrameIndex == 1)
        {
            if (byte != IBUS_HEADER_1)
            {
                ibusFrameIndex = 0;
                continue;
            }

            ibusFrame[ibusFrameIndex++] = byte;
            continue;
        }

        ibusFrame[ibusFrameIndex++] = byte;

        if (ibusFrameIndex >= IBUS_FRAME_LENGTH)
        {
            uint16_t checksum = 0xFFFF;

            for (uint8_t i = 0; i < 30; i++)
            {
                checksum -= ibusFrame[i];
            }

            uint16_t receivedChecksum =
                ibusFrame[30] |
                (ibusFrame[31] << 8);

            if (checksum == receivedChecksum)
            {
                for (uint8_t channel = 0; channel < IBUS_CHANNELS; channel++)
                {
                    ibusChannels[channel] =
                        ibusFrame[2 + channel * 2] |
                        (ibusFrame[3 + channel * 2] << 8);
                }

                ibusLastFrame = micros();
                ibusFrameReady = true;
            }

            ibusFrameIndex = 0;
        }
    }
}

// ============================================================
// COPY IBUS DATA
// ============================================================

void getIBusChannels(uint16_t *destination)
{
    for (uint8_t i = 0; i < IBUS_CHANNELS; i++)
    {
        destination[i] = ibusChannels[i];
    }
}

// ============================================================
// CLAMP IBUS VALUE
// ============================================================

uint16_t clampIBus(uint16_t value)
{
    return constrain(value, PWM_MIN, PWM_MAX);
}

// ============================================================

// MAP CONTROL INPUT

// ============================================================

int16_t centeredControl(

    uint16_t input,

    int16_t maximumDeflection,

    bool reverse = false

)

{

    input = clampIBus(input);

    int32_t output =

        map(

            input,

            1000,

            2000,

            -maximumDeflection,

            maximumDeflection

        );

    if (reverse) {

        output = -output;

    }

    return constrain(

        output,

        -maximumDeflection,

        maximumDeflection

    );

}

// ============================================================
// SET SAFE OUTPUTS
// ============================================================

void setSafeOutputs()
{
    servoAileronLeft.writeMicroseconds(PWM_CENTER);
    servoAileronRight.writeMicroseconds(PWM_CENTER);

    servoElevator.writeMicroseconds(PWM_CENTER);

    // ESC receives minimum throttle.

    esc.writeMicroseconds(PWM_MIN);
}

// ============================================================

// ARM LOGIC

// ============================================================

void updateArming(uint16_t throttle)

{

}

// ============================================================

// CONTROL UPDATE

// ============================================================

void updateControls()

{

uint16_t ch[IBUS_CHANNELS];

getIBusChannels(ch);

uint16_t aileronInput = ch[0];
uint16_t elevatorInput = ch[1];
uint16_t throttleInput = ch[2];

// Additional channels available:
//
// CH4 = Rudder
// CH5 = Gear
// CH6 = Flaps / auxiliary
// CH7 = Auxiliary
// CH8 = Auxiliary
// CH9 = Auxiliary
// CH10 = Auxiliary

    // --------------------------------------------------------

    // RECEIVER FAILSAFE

    // --------------------------------------------------------

uint32_t now = micros();

uint32_t lastFrame = ibusLastFrame;

receiverFailsafe =
    ((now - lastFrame) > RX_TIMEOUT_US);

    // --------------------------------------------------------

    // FAILSAFE OUTPUT

    // --------------------------------------------------------

    if (receiverFailsafe) {

        armed = false;

        setSafeOutputs();

        return;

    }

    // --------------------------------------------------------

    // ARMING

    // --------------------------------------------------------

    updateArming(throttleInput);

    // --------------------------------------------------------

    // CONTROL SURFACES

    // --------------------------------------------------------

// --------------------------------------------------------
// CONTROL SURFACES
// --------------------------------------------------------

int16_t aileron =
    centeredControl(
        aileronInput,
        AILERON_MAX_US,
        false
    );

int16_t elevator =
    centeredControl(
        elevatorInput,
        ELEVATOR_MAX_US,
        false
    );

// Left and right ailerons move in opposite directions.

uint16_t aileronLeftOutput =
    constrain(
        PWM_CENTER + aileron,
        PWM_MIN,
        PWM_MAX
    );

uint16_t aileronRightOutput =
    constrain(
        PWM_CENTER - aileron,
        PWM_MIN,
        PWM_MAX
    );

uint16_t elevatorOutput =
    constrain(
        PWM_CENTER + elevator,
        PWM_MIN,
        PWM_MAX
    );

servoAileronLeft.writeMicroseconds(aileronLeftOutput);
servoAileronRight.writeMicroseconds(aileronRightOutput);

servoElevator.writeMicroseconds(elevatorOutput);

    // --------------------------------------------------------

    // MOTOR SAFETY

    // --------------------------------------------------------

   // --------------------------------------------------------
// MOTOR (CH3 → 1000-2000 µs = 0-100 %)
// --------------------------------------------------------
uint16_t throttle =
    constrain(
        throttleInput,
        PWM_MIN,
        PWM_MAX
    );
esc.writeMicroseconds(throttle);

}

// ============================================================

// DEBUG OUTPUT

// ============================================================

// ============================================================
// DEBUG OUTPUT
// ============================================================

void debugOutput()
{
    static uint32_t lastDebug = 0;

    if (millis() - lastDebug < DEBUG_INTERVAL_MS)
    {
        return;
    }

    lastDebug = millis();

    uint16_t ch[IBUS_CHANNELS];

    getIBusChannels(ch);

    Serial.print("IBUS: ");

    for (uint8_t i = 0; i < IBUS_CHANNELS; i++)
    {
        Serial.print("CH");
        Serial.print(i + 1);
        Serial.print("=");
        Serial.print(ch[i]);
        Serial.print(" ");
    }

    Serial.print("| RX=");

    if (receiverFailsafe)
    {
        Serial.print("LOST");
    }
    else
    {
        Serial.print("OK");
    }

    Serial.print(" | ARM=");
    Serial.print(armed ? "YES" : "NO");

    Serial.print(" | OUT LAIL=");
    Serial.print(servoAileronLeft.readMicroseconds());

    Serial.print(" RAIL=");
    Serial.print(servoAileronRight.readMicroseconds());

    Serial.print(" ELE=");
    Serial.print(servoElevator.readMicroseconds());

    Serial.print(" ESC=");
    Serial.println(esc.readMicroseconds());
}

// ============================================================

// SETUP

// ============================================================

void setup()

{

    Serial.begin(115200);

    delay(1000);

    Serial.println();

    Serial.println("=================================");

    Serial.println(" AEROS-001 FLIGHT CONTROLLER");

    Serial.println(" ESP32-C3");

    Serial.println("=================================");

    Serial.println();


    // --------------------------------------------------------

    // SERVO PWM

    // --------------------------------------------------------

    ESP32PWM::allocateTimer(0);

    ESP32PWM::allocateTimer(1);

    ESP32PWM::allocateTimer(2);

    ESP32PWM::allocateTimer(3);
    servoAileronLeft.setPeriodHertz(50);
    servoAileronRight.setPeriodHertz(50);

    servoElevator.setPeriodHertz(50);

    esc.setPeriodHertz(50);


    bool aileronLeftOK =
    servoAileronLeft.attach(
        PIN_AILERON_LEFT,
        PWM_MIN,
        PWM_MAX
    );

bool aileronRightOK =
    servoAileronRight.attach(
        PIN_AILERON_RIGHT,
        PWM_MIN,
        PWM_MAX
    );

bool elevatorOK =
    servoElevator.attach(
        PIN_ELEVATOR,
        PWM_MIN,
        PWM_MAX
    );

bool escOK =
    esc.attach(
        PIN_ESC,
        PWM_MIN,
        PWM_MAX
    );

    Serial.print("Aileron LEFT attach: ");

    Serial.println(
        aileronLeftOK ? "OK" : "FAILED"
    );

    Serial.print("Aileron RIGHT attach: ");

    Serial.println(
        aileronRightOK ? "OK" : "FAILED"
    );

    Serial.print("Elevator attach: ");

    Serial.println(

        elevatorOK ? "OK" : "FAILED"

    );

    Serial.print("ESC attach: ");

    Serial.println(

        escOK ? "OK" : "FAILED"

    );

    // --------------------------------------------------------

    // INITIAL SAFE OUTPUT

    // --------------------------------------------------------

    setSafeOutputs();

    // --------------------------------------------------------
// IBUS INPUT
// --------------------------------------------------------

IBusSerial.begin(
    IBUS_BAUDRATE,
    SERIAL_8N1,
    PIN_IBUS,
    -1
);

ibusLastFrame = micros();

Serial.println();

Serial.println("iBUS input initialized.");
Serial.println("115200 baud.");
Serial.println("10 channels.");
Serial.println("Throttle must be LOW.");
Serial.println("Motor is DISARMED.");
Serial.println();

}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    readIBus();

    updateControls();

    debugOutput();

    delay(2);
}