#include <Arduino.h>
#include <ESP32Servo.h>

// ============================================================
// AEROS-001 FLIGHT CONTROLLER
// ESP32-C3 SuperMini
//
// Текущая архитектура:
//
//   Receiver
//       ↓
//   FlightController
//       ↓
//   ControlMixer
//       ↓
//   ThrottleManager
//       ↓
//   FlightOutputs
//
// В будущем сюда можно независимо добавить:
//
//   IMU
//   Gyroscope
//   Accelerometer
//   GPS
//   Autopilot
//   WaypointNavigator
//   Telemetry
//   GUI
//   FlightModes
//
// Весь код пока находится в одном файле намеренно.
// Классы уже изолированы так, чтобы позже их можно было
// безболезненно разнести по отдельным .h/.cpp.
// ============================================================


// ============================================================
// 1. CONFIGURATION
// Все постоянные параметры проекта находятся здесь.
// Логика классов ниже не должна содержать "магических" пинов,
// таймаутов и прочих настроек.
// ============================================================

namespace Config
{
    // --------------------------------------------------------
    // Hardware pinout
    // --------------------------------------------------------

    constexpr uint8_t PIN_AILERON_LEFT  = 5;
    constexpr uint8_t PIN_AILERON_RIGHT = 4;
    constexpr uint8_t PIN_ELEVATOR      = 6;
    constexpr uint8_t PIN_ESC            = 7;
    constexpr uint8_t PIN_IBUS           = 8;


    // --------------------------------------------------------
    // iBUS configuration
    // --------------------------------------------------------

    constexpr uint8_t IBUS_CHANNELS = 10;
    constexpr uint8_t IBUS_FRAME_LENGTH = 32;

    constexpr uint8_t IBUS_HEADER_0 = 0x20;
    constexpr uint8_t IBUS_HEADER_1 = 0x40;

    constexpr uint32_t IBUS_BAUDRATE = 115200;

    // При отсутствии корректного iBUS кадра дольше этого
    // времени приёмник считается потерянным.
    constexpr uint32_t RX_TIMEOUT_US = 100000;


    // --------------------------------------------------------
    // Standard RC pulse range
    // --------------------------------------------------------

    constexpr uint16_t PWM_MIN    = 1000;
    constexpr uint16_t PWM_CENTER = 1500;
    constexpr uint16_t PWM_MAX    = 2000;


    // --------------------------------------------------------
    // Flight-control limits
    // --------------------------------------------------------

    // Максимальное отклонение элеронов относительно центра.
    constexpr int16_t AILERON_MAX_US = 1500;

    // Максимальное отклонение руля высоты.
    constexpr int16_t ELEVATOR_MAX_US = 1000;


    // --------------------------------------------------------
    // Throttle safety
    // --------------------------------------------------------

    // Ниже этого значения газ считается LOW.
    constexpr uint16_t THROTTLE_LOW_US = 1050;

    // Минимальное время нахождения газа в LOW.
    constexpr uint32_t ARM_LOW_TIME_MS = 1500;


    // --------------------------------------------------------
    // Failsafe outputs
    // --------------------------------------------------------

    constexpr uint16_t FAILSAFE_AILERON  = 1500;
    constexpr uint16_t FAILSAFE_ELEVATOR = 1500;
    constexpr uint16_t FAILSAFE_THROTTLE = 1000;


    // --------------------------------------------------------
    // Throttle boost
    // --------------------------------------------------------

    // В обычном режиме максимальный газ = 40%.
    constexpr uint16_t THROTTLE_LIMIT_PERCENT = 40;

    // Полный газ разрешается на 5 секунд.
    constexpr uint32_t THROTTLE_BOOST_TIME_MS = 5000;


    // --------------------------------------------------------
    // Debug
    // --------------------------------------------------------

    constexpr uint32_t DEBUG_INTERVAL_MS = 100;
}


// ============================================================
// 2. CHANNEL MAP
//
// Важно: здесь находится только логическое описание каналов.
// Если позже передатчик будет перенастроен, менять нужно будет
// только этот блок.
// ============================================================

namespace Channels
{
    constexpr uint8_t AILERON  = 0;  // CH1
    constexpr uint8_t ELEVATOR = 1;  // CH2
    constexpr uint8_t THROTTLE = 2;  // CH3
    constexpr uint8_t RUDDER   = 3;  // CH4

    constexpr uint8_t FLAPS    = 4;  // CH5

    constexpr uint8_t AUX_1    = 5;  // CH6
    constexpr uint8_t AUX_2    = 6;  // CH7

    // Текущая логика boost использует CH8.
    constexpr uint8_t BOOST    = 7;  // CH8

    constexpr uint8_t AUX_4    = 8;  // CH9
    constexpr uint8_t AUX_5    = 9;  // CH10
}


// ============================================================
// 3. RC CHANNEL STATE
//
// Этот класс представляет состояние одного набора каналов.
// Никакой логики управления самолётом здесь нет.
//
// В будущем сюда можно будет добавить:
// - timestamp;
// - quality;
// - signal strength;
// - channel validity;
// - failsafe information.
// ============================================================

class RcChannelState
{
public:

    RcChannelState()
    {
        reset();
    }


    // --------------------------------------------------------
    // Сброс каналов в безопасное состояние.
    // --------------------------------------------------------

    void reset()
    {
        for (uint8_t i = 0; i < Config::IBUS_CHANNELS; ++i)
        {
            channels[i] = Config::PWM_CENTER;
        }

        channels[Channels::THROTTLE] = Config::PWM_MIN;
    }


    // --------------------------------------------------------
    // Получить значение конкретного канала.
    // --------------------------------------------------------

    uint16_t get(uint8_t index) const
    {
        if (index >= Config::IBUS_CHANNELS)
        {
            return Config::PWM_CENTER;
        }

        return channels[index];
    }


    // --------------------------------------------------------
    // Установить значение конкретного канала.
    // --------------------------------------------------------

    void set(uint8_t index, uint16_t value)
    {
        if (index >= Config::IBUS_CHANNELS)
        {
            return;
        }

        channels[index] = value;
    }


    // --------------------------------------------------------
    // Доступ к массиву каналов.
    //
    // Используется только для систем, которым действительно
    // нужен весь набор каналов.
    // --------------------------------------------------------

    const uint16_t* data() const
    {
        return channels;
    }


private:

    uint16_t channels[Config::IBUS_CHANNELS];
};


// ============================================================
// 4. IBUS RECEIVER
//
// Единственная задача класса:
//
// UART → iBUS frames → RC channel state
//
// Здесь НЕТ:
// - сервоприводов;
// - throttle;
// - failsafe поведения самолёта;
// - mixer;
// - автопилота.
//
// Благодаря этому в будущем можно заменить iBUS на другой
// источник команд, не переписывая FlightController.
// ============================================================

class IBusReceiver
{
public:

    explicit IBusReceiver(HardwareSerial& serial)
        : serial(serial)
    {
    }


    // --------------------------------------------------------
    // Инициализация UART.
    // --------------------------------------------------------

    void begin()
    {
        serial.begin(
            Config::IBUS_BAUDRATE,
            SERIAL_8N1,
            Config::PIN_IBUS,
            -1
        );

        lastFrameTime = micros();
    }


    // --------------------------------------------------------
    // Обработка входящих UART данных.
    //
    // Метод должен вызываться постоянно из loop().
    // --------------------------------------------------------

    void update()
    {
        while (serial.available())
        {
            const uint8_t byte = serial.read();

            processByte(byte);
        }
    }


    // --------------------------------------------------------
    // Возвращает текущее состояние каналов.
    // --------------------------------------------------------

    const RcChannelState& getState() const
    {
        return state;
    }


    // --------------------------------------------------------
    // Проверка наличия свежего iBUS кадра.
    // --------------------------------------------------------

    bool isSignalLost() const
    {
        return (micros() - lastFrameTime) > Config::RX_TIMEOUT_US;
    }


    // --------------------------------------------------------
    // Время последнего корректного кадра.
    // --------------------------------------------------------

    uint32_t getLastFrameTime() const
    {
        return lastFrameTime;
    }


private:

    HardwareSerial& serial;

    RcChannelState state;

    uint8_t frame[Config::IBUS_FRAME_LENGTH] = {};

    uint8_t frameIndex = 0;

    uint32_t lastFrameTime = 0;


    // --------------------------------------------------------
    // Обработка одного байта входящего iBUS потока.
    //
    // Состояния:
    //
    // 0 → ждём 0x20
    // 1 → ждём 0x40
    // 2..31 → принимаем frame
    // --------------------------------------------------------

    void processByte(uint8_t byte)
    {
        // ----------------------------------------------------
        // Поиск первого байта заголовка.
        // ----------------------------------------------------

        if (frameIndex == 0)
        {
            if (byte != Config::IBUS_HEADER_0)
            {
                return;
            }

            frame[frameIndex++] = byte;
            return;
        }


        // ----------------------------------------------------
        // Проверка второго байта заголовка.
        // ----------------------------------------------------

        if (frameIndex == 1)
        {
            if (byte != Config::IBUS_HEADER_1)
            {
                frameIndex = 0;
                return;
            }

            frame[frameIndex++] = byte;
            return;
        }


        // ----------------------------------------------------
        // Принимаем оставшуюся часть кадра.
        // ----------------------------------------------------

        frame[frameIndex++] = byte;


        // ----------------------------------------------------
        // Полный кадр получен.
        // ----------------------------------------------------

        if (frameIndex >= Config::IBUS_FRAME_LENGTH)
        {
            processFrame();

            frameIndex = 0;
        }
    }


    // --------------------------------------------------------
    // Проверка checksum и извлечение каналов.
    // --------------------------------------------------------

    void processFrame()
    {
        uint16_t checksum = 0xFFFF;

        // Первые 30 байт участвуют в checksum.
        for (uint8_t i = 0; i < 30; ++i)
        {
            checksum -= frame[i];
        }


        // ----------------------------------------------------
        // Checksum, переданный приёмником.
        // ----------------------------------------------------

        const uint16_t receivedChecksum =
            static_cast<uint16_t>(frame[30]) |
            (static_cast<uint16_t>(frame[31]) << 8);


        // ----------------------------------------------------
        // Некорректный кадр игнорируем.
        // ----------------------------------------------------

        if (checksum != receivedChecksum)
        {
            return;
        }


        // ----------------------------------------------------
        // Извлекаем 10 каналов.
        // ----------------------------------------------------

        for (uint8_t channel = 0;
             channel < Config::IBUS_CHANNELS;
             ++channel)
        {
            const uint8_t lowByte  = frame[2 + channel * 2];
            const uint8_t highByte = frame[3 + channel * 2];

            const uint16_t value =
                static_cast<uint16_t>(lowByte) |
                (static_cast<uint16_t>(highByte) << 8);

            state.set(channel, value);
        }


        // ----------------------------------------------------
        // Фиксируем время последнего валидного кадра.
        // ----------------------------------------------------

        lastFrameTime = micros();
    }
};


// ============================================================
// 5. RC INPUT UTILITIES
//
// Маленький независимый класс для стандартной обработки
// значений RC.
//
// Здесь нет знания о самолёте.
// ============================================================

class RcInput
{
public:

    // --------------------------------------------------------
    // Ограничение RC значения стандартным диапазоном.
    // --------------------------------------------------------

    static uint16_t clamp(uint16_t value)
    {
        return constrain(
            value,
            Config::PWM_MIN,
            Config::PWM_MAX
        );
    }


    // --------------------------------------------------------
    // Преобразование:
//
// 1000 → -maximumDeflection
// 1500 → 0
// 2000 → +maximumDeflection
//
// reverse позволяет инвертировать канал.
// --------------------------------------------------------

    static int16_t centered(
        uint16_t input,
        int16_t maximumDeflection,
        bool reverse = false
    )
    {
        input = clamp(input);

        int32_t output = map(
            input,
            Config::PWM_MIN,
            Config::PWM_MAX,
            -maximumDeflection,
            maximumDeflection
        );


        if (reverse)
        {
            output = -output;
        }


        return static_cast<int16_t>(
            constrain(
                output,
                -maximumDeflection,
                maximumDeflection
            )
        );
    }
};


// ============================================================
// 6. FLIGHT OUTPUT STATE
//
// Это логическое представление того, что мы хотим отправить
// на физические исполнительные механизмы.
//
// Важный момент:
//
// ControlMixer НЕ должен знать о Servo.
//
// Он только рассчитывает:
//
//   left aileron
//   right aileron
//   elevator
//   throttle
//
// А FlightOutputs уже превращает это в PWM.
// ============================================================

struct FlightOutputState
{
    uint16_t aileronLeft  = Config::PWM_CENTER;
    uint16_t aileronRight = Config::PWM_CENTER;
    uint16_t elevator     = Config::PWM_CENTER;
    uint16_t throttle     = Config::PWM_MIN;
};


// ============================================================
// 7. CONTROL MIXER
//
// Здесь находится только аэродинамическая логика.
//
// RC:
//
// CH1 → Aileron
// CH2 → Elevator
// CH5 → Flaps
//
// На выходе:
//
// Left Aileron
// Right Aileron
// Elevator
//
// Никакого UART.
// Никаких Servo.
// Никакого failsafe.
// Никакого millis().
//
// Это особенно важно для будущего автопилота:
//
// manual input и autopilot output смогут использовать
// один и тот же mixer.
// ============================================================

class ControlMixer
{
public:

    // --------------------------------------------------------
    // Расчёт управляющих поверхностей.
    // --------------------------------------------------------

    FlightOutputState calculate(
        const RcChannelState& rc
    ) const
    {
        FlightOutputState output;


        // ----------------------------------------------------
        // Получаем основные RC inputs.
        // ----------------------------------------------------

        const uint16_t aileronInput =
            rc.get(Channels::AILERON);

        const uint16_t elevatorInput =
            rc.get(Channels::ELEVATOR);


        // ----------------------------------------------------
        // Преобразуем Aileron.
        // ----------------------------------------------------

        const int16_t aileron =
            RcInput::centered(
                aileronInput,
                Config::AILERON_MAX_US,
                false
            );


        // ----------------------------------------------------
        // Преобразуем Elevator.
        // ----------------------------------------------------

        const int16_t elevator =
            RcInput::centered(
                elevatorInput,
                Config::ELEVATOR_MAX_US,
                false
            );


        // ----------------------------------------------------
        // Рассчитываем положение закрылков.
        //
        // CH5:
        //
        // < 1250 → 0 us
        // 1250..1749 → 50 us
        // >= 1750 → 100 us
        // ----------------------------------------------------

        const uint16_t flapOffset =
            calculateFlapOffset(
                rc.get(Channels::FLAPS)
            );


        // ----------------------------------------------------
        // LEFT AILERON
        //
        // Элерон + flap offset.
        // ----------------------------------------------------

        int32_t left =
            Config::PWM_CENTER +
            aileron +
            flapOffset;


        // ----------------------------------------------------
        // RIGHT AILERON
        //
        // Элерон зеркальный.
        //
        // Flap offset остаётся физически направленным вниз
        // относительно соответствующего крыла.
        // ----------------------------------------------------

        int32_t right =
            Config::PWM_CENTER -
            aileron -
            flapOffset;


        // ----------------------------------------------------
        // Ограничиваем выходы стандартным PWM диапазоном.
        // ----------------------------------------------------

        left = constrain(
            left,
            Config::PWM_MIN,
            Config::PWM_MAX
        );

        right = constrain(
            right,
            Config::PWM_MIN,
            Config::PWM_MAX
        );


        // ----------------------------------------------------
        // Elevator.
        // ----------------------------------------------------

        const int32_t elevatorOutput =
            constrain(
                Config::PWM_CENTER + elevator,
                Config::PWM_MIN,
                Config::PWM_MAX
            );


        // ----------------------------------------------------
        // Формируем итоговое состояние поверхностей.
        // ----------------------------------------------------

        output.aileronLeft =
            static_cast<uint16_t>(left);

        output.aileronRight =
            static_cast<uint16_t>(right);

        output.elevator =
            static_cast<uint16_t>(elevatorOutput);


        return output;
    }


private:

    // --------------------------------------------------------
    // Преобразование положения CH5 в flap offset.
    // --------------------------------------------------------

    uint16_t calculateFlapOffset(uint16_t input) const
    {
        if (input >= 1750)
        {
            return 100;
        }

        if (input >= 1250)
        {
            return 50;
        }

        return 0;
    }
};


// ============================================================
// 8. THROTTLE MANAGER
//
// Вся логика двигателя находится здесь.
//
// Ответственность:
//
// - чтение throttle;
// - ограничение мощности;
// - boost;
// - boost timer;
// - повторная активация boost.
//
// Этот класс ничего не знает о Servo.
// Он просто возвращает требуемый PWM.
// ============================================================

class ThrottleManager
{
public:

    // --------------------------------------------------------
    // Обновление throttle.
    // --------------------------------------------------------

    uint16_t update(
        const RcChannelState& rc,
        bool receiverFailsafe
    )
    {
        const uint32_t now = millis();


        // ----------------------------------------------------
        // Приёмник потерян → немедленно выключаем boost.
        // ----------------------------------------------------

        if (receiverFailsafe)
        {
            boostActive = false;

            return Config::FAILSAFE_THROTTLE;
        }


        const uint16_t throttle =
            RcInput::clamp(
                rc.get(Channels::THROTTLE)
            );


        const uint16_t boostSwitch =
            rc.get(Channels::BOOST);


        // ----------------------------------------------------
        // LOW на boost switch снова разрешает следующий boost.
        // ----------------------------------------------------

        if (boostSwitch < 1250)
        {
            boostReady = true;
        }


        // ----------------------------------------------------
        // HIGH на boost switch запускает boost.
        //
        // Boost запускается только один раз до тех пор,
        // пока переключатель не вернётся в LOW.
        // ----------------------------------------------------

        if (
            boostSwitch >= 1750 &&
            boostReady &&
            !boostActive
        )
        {
            boostActive = true;
            boostReady = false;
            boostStartTime = now;
        }


        // ----------------------------------------------------
        // Проверяем таймер boost.
        // ----------------------------------------------------

        if (boostActive)
        {
            if (
                now - boostStartTime >=
                Config::THROTTLE_BOOST_TIME_MS
            )
            {
                boostActive = false;
            }
        }


        // ----------------------------------------------------
        // Во время boost разрешаем полный газ.
        // ----------------------------------------------------

        if (boostActive)
        {
            return Config::PWM_MAX;
        }


        // ----------------------------------------------------
        // Обычный режим.
        //
        // 1000 → 1000
        // 1500 → 1200
        // 2000 → 1400
        //
        // То есть весь ход стика сохраняется,
        // но максимум ограничивается 40%.
        // ----------------------------------------------------

        const uint16_t maximumThrottle =
            Config::PWM_MIN +
            (
                (Config::PWM_MAX - Config::PWM_MIN) *
                Config::THROTTLE_LIMIT_PERCENT
            ) / 100;


        uint16_t limitedThrottle =
            map(
                throttle,
                Config::PWM_MIN,
                Config::PWM_MAX,
                Config::PWM_MIN,
                maximumThrottle
            );


        // ----------------------------------------------------
        // Дополнительная защита результата.
        // ----------------------------------------------------

        limitedThrottle =
            constrain(
                limitedThrottle,
                Config::PWM_MIN,
                maximumThrottle
            );


        return limitedThrottle;
    }


    // --------------------------------------------------------
    // Активен ли boost прямо сейчас.
    // --------------------------------------------------------

    bool isBoostActive() const
    {
        return boostActive;
    }


    // --------------------------------------------------------
    // Можно ли снова запустить boost.
    // --------------------------------------------------------

    bool isBoostReady() const
    {
        return boostReady;
    }


private:

    bool boostActive = false;

    bool boostReady = true;

    uint32_t boostStartTime = 0;
};


// ============================================================
// 9. ARMING MANAGER
//
// В исходном коде функция updateArming() была пустой,
// поэтому armed фактически никогда не становился true.
//
// Здесь логика вынесена отдельно.
//
// ВАЖНО:
//
// Сейчас armed НЕ блокирует throttle, потому что это изменило
// бы исходное поведение программы.
//
// Этот класс пока только ведёт состояние ARM.
//
// Когда появится полноценная arm/disarm логика, её можно будет
// изменить здесь, не трогая receiver/mixer/outputs.
// ============================================================

class ArmingManager
{
public:

    // --------------------------------------------------------
    // Обновление состояния ARM.
    // --------------------------------------------------------

    void update(
        uint16_t throttle,
        bool receiverFailsafe
    )
    {
        // ----------------------------------------------------
        // Приёмник потерян → DISARM.
        // ----------------------------------------------------

        if (receiverFailsafe)
        {
            armed = false;
            throttleLowSince = 0;
            return;
        }


        // ----------------------------------------------------
        // Газ LOW.
        // ----------------------------------------------------

        if (throttle < Config::THROTTLE_LOW_US)
        {
            if (throttleLowSince == 0)
            {
                throttleLowSince = millis();
            }


            // ------------------------------------------------
            // После удержания газа LOW считаем систему ARM-ready.
            // ------------------------------------------------

            if (
                millis() - throttleLowSince >=
                Config::ARM_LOW_TIME_MS
            )
            {
                armed = true;
            }

            return;
        }


        // ----------------------------------------------------
        // Газ поднят.
        //
        // В текущей архитектуре не делаем автоматический
        // disarm здесь, чтобы не менять поведение оригинала.
        // ----------------------------------------------------

        throttleLowSince = 0;
    }


    // --------------------------------------------------------
    // Текущее состояние ARM.
    // --------------------------------------------------------

    bool isArmed() const
    {
        return armed;
    }


private:

    bool armed = false;

    uint32_t throttleLowSince = 0;
};


// ============================================================
// 10. FLIGHT OUTPUTS
//
// Единственный класс, который знает о Servo.
//
// Это очень важная граница.
//
// Если позже вместо ESP32Servo появится:
// - другой PWM driver;
// - PCA9685;
// - другой MCU;
// - simulator;
//
// остальные классы менять не придётся.
// ============================================================

class FlightOutputs
{
public:

    // --------------------------------------------------------
    // Инициализация PWM.
    // --------------------------------------------------------

    bool begin()
    {
        // ----------------------------------------------------
        // Выделяем все четыре hardware timers ESP32Servo.
        // ----------------------------------------------------

        ESP32PWM::allocateTimer(0);
        ESP32PWM::allocateTimer(1);
        ESP32PWM::allocateTimer(2);
        ESP32PWM::allocateTimer(3);


        // ----------------------------------------------------
        // Все поверхности и ESC работают на 50 Hz.
        // ----------------------------------------------------

        aileronLeft.setPeriodHertz(50);
        aileronRight.setPeriodHertz(50);
        elevator.setPeriodHertz(50);
        esc.setPeriodHertz(50);


        // ----------------------------------------------------
        // Подключаем левый элерон.
        // ----------------------------------------------------

        const bool leftOK =
            aileronLeft.attach(
                Config::PIN_AILERON_LEFT,
                Config::PWM_MIN,
                Config::PWM_MAX
            );


        // ----------------------------------------------------
        // Подключаем правый элерон.
        // ----------------------------------------------------

        const bool rightOK =
            aileronRight.attach(
                Config::PIN_AILERON_RIGHT,
                Config::PWM_MIN,
                Config::PWM_MAX
            );


        // ----------------------------------------------------
        // Подключаем elevator.
        // ----------------------------------------------------

        const bool elevatorOK =
            elevator.attach(
                Config::PIN_ELEVATOR,
                Config::PWM_MIN,
                Config::PWM_MAX
            );


        // ----------------------------------------------------
        // Подключаем ESC.
        // ----------------------------------------------------

        const bool escOK =
            esc.attach(
                Config::PIN_ESC,
                Config::PWM_MIN,
                Config::PWM_MAX
            );


        return
            leftOK &&
            rightOK &&
            elevatorOK &&
            escOK;
    }


    // --------------------------------------------------------
    // Применить рассчитанное состояние к физическим Servo.
    // --------------------------------------------------------

    void write(const FlightOutputState& state)
    {
        aileronLeft.writeMicroseconds(
            state.aileronLeft
        );

        aileronRight.writeMicroseconds(
            state.aileronRight
        );

        elevator.writeMicroseconds(
            state.elevator
        );

        esc.writeMicroseconds(
            state.throttle
        );


        // ----------------------------------------------------
        // Сохраняем последнее состояние для debug/telemetry.
        // ----------------------------------------------------

        lastState = state;
    }


    // --------------------------------------------------------
    // Немедленно выставить безопасные выходы.
    // --------------------------------------------------------

    void setFailsafe()
    {
        FlightOutputState safe;

        safe.aileronLeft =
            Config::FAILSAFE_AILERON;

        safe.aileronRight =
            Config::FAILSAFE_AILERON;

        safe.elevator =
            Config::FAILSAFE_ELEVATOR;

        safe.throttle =
            Config::FAILSAFE_THROTTLE;


        write(safe);
    }


    // --------------------------------------------------------
    // Получить последние записанные значения.
    // --------------------------------------------------------

    const FlightOutputState& getLastState() const
    {
        return lastState;
    }


private:

    Servo aileronLeft;
    Servo aileronRight;
    Servo elevator;
    Servo esc;

    FlightOutputState lastState;
};


// ============================================================
// 11. FLIGHT CONTROLLER
//
// Это главный координатор.
//
// Очень важно:
//
// FlightController НЕ содержит реализацию iBUS.
// FlightController НЕ управляет Servo напрямую.
// FlightController НЕ считает mixer вручную.
// FlightController НЕ занимается debug.
//
// Он только координирует подсистемы:
//
// Receiver
//   ↓
// Failsafe
//   ↓
// Arming
//   ↓
// Mixer
//   ↓
// Throttle
//   ↓
// Outputs
//
// Именно этот класс в будущем станет точкой объединения
// ручного управления и автопилота.
// ============================================================

class FlightController
{
public:

    FlightController(
        IBusReceiver& receiver,
        ControlMixer& mixer,
        ThrottleManager& throttle,
        ArmingManager& arming,
        FlightOutputs& outputs
    )
        : receiver(receiver),
          mixer(mixer),
          throttle(throttle),
          arming(arming),
          outputs(outputs)
    {
    }


    // --------------------------------------------------------
    // Основная инициализация.
    // --------------------------------------------------------

    void begin()
    {
        outputs.setFailsafe();

        receiver.begin();
    }


    // --------------------------------------------------------
    // Главный цикл flight controller.
    // --------------------------------------------------------

    void update()
    {
        // ----------------------------------------------------
        // 1. Получаем новые iBUS кадры.
        // ----------------------------------------------------

        receiver.update();


        // ----------------------------------------------------
        // 2. Проверяем состояние радиоканала.
        // ----------------------------------------------------

        const bool receiverFailsafe =
            receiver.isSignalLost();


        // ----------------------------------------------------
        // 3. Получаем текущее состояние RC.
        // ----------------------------------------------------

        const RcChannelState& rc =
            receiver.getState();


        // ----------------------------------------------------
        // 4. Failsafe имеет абсолютный приоритет.
        // ----------------------------------------------------

        if (receiverFailsafe)
        {
            arming.update(
                Config::PWM_MIN,
                true
            );

            throttle.update(
                rc,
                true
            );

            outputs.setFailsafe();

            return;
        }


        // ----------------------------------------------------
        // 5. Обновляем ARM state.
        // ----------------------------------------------------

        arming.update(
            rc.get(Channels::THROTTLE),
            false
        );


        // ----------------------------------------------------
        // 6. Рассчитываем поверхности управления.
        // ----------------------------------------------------

        FlightOutputState output =
            mixer.calculate(rc);


        // ----------------------------------------------------
        // 7. Рассчитываем throttle.
        // ----------------------------------------------------

        output.throttle =
            throttle.update(
                rc,
                false
            );


        // ----------------------------------------------------
        // 8. Отправляем весь рассчитанный state
        //    физическим выходам.
        // ----------------------------------------------------

        outputs.write(output);
    }


    // --------------------------------------------------------
    // Состояние приёмника.
    // --------------------------------------------------------

    bool isReceiverFailsafe() const
    {
        return receiver.isSignalLost();
    }


    // --------------------------------------------------------
    // Состояние ARM.
    // --------------------------------------------------------

    bool isArmed() const
    {
        return arming.isArmed();
    }


    // --------------------------------------------------------
    // Состояние boost.
    // --------------------------------------------------------

    bool isBoostActive() const
    {
        return throttle.isBoostActive();
    }


    // --------------------------------------------------------
    // Последние PWM outputs.
    // --------------------------------------------------------

    const FlightOutputState& getOutputState() const
    {
        return outputs.getLastState();
    }


    // --------------------------------------------------------
    // Текущее RC состояние.
    // --------------------------------------------------------

    const RcChannelState& getRcState() const
    {
        return receiver.getState();
    }


private:

    IBusReceiver& receiver;

    ControlMixer& mixer;

    ThrottleManager& throttle;

    ArmingManager& arming;

    FlightOutputs& outputs;
};


// ============================================================
// 12. DEBUG LOGGER
//
// Debug полностью отделён от flight logic.
//
// В будущем этот класс можно заменить на:
//
// SerialLogger
// TelemetryLogger
// WiFiLogger
// WebSocketLogger
// SDLogger
//
// При этом FlightController менять не потребуется.
// ============================================================

class DebugLogger
{
public:

    explicit DebugLogger(
        FlightController& controller
    )
        : controller(controller)
    {
    }


    // --------------------------------------------------------
    // Периодический вывод состояния.
    // --------------------------------------------------------

    void update()
    {
        const uint32_t now = millis();

        if (
            now - lastDebugTime <
            Config::DEBUG_INTERVAL_MS
        )
        {
            return;
        }


        lastDebugTime = now;

        printState();
    }


private:

    FlightController& controller;

    uint32_t lastDebugTime = 0;


    // --------------------------------------------------------
    // Вывод полного текущего состояния.
    // --------------------------------------------------------

    void printState()
    {
        const RcChannelState& rc =
            controller.getRcState();

        const FlightOutputState& output =
            controller.getOutputState();


        // ----------------------------------------------------
        // RC channels.
        // ----------------------------------------------------

        Serial.print("IBUS: ");

        for (
            uint8_t i = 0;
            i < Config::IBUS_CHANNELS;
            ++i
        )
        {
            Serial.print("CH");
            Serial.print(i + 1);
            Serial.print("=");

            Serial.print(rc.get(i));

            Serial.print(" ");
        }


        // ----------------------------------------------------
        // Receiver status.
        // ----------------------------------------------------

        Serial.print("| RX=");

        Serial.print(
            controller.isReceiverFailsafe()
                ? "LOST"
                : "OK"
        );


        // ----------------------------------------------------
        // ARM status.
        // ----------------------------------------------------

        Serial.print(" | ARM=");

        Serial.print(
            controller.isArmed()
                ? "YES"
                : "NO"
        );


        // ----------------------------------------------------
        // Boost status.
        // ----------------------------------------------------

        Serial.print(" | BOOST=");

        Serial.print(
            controller.isBoostActive()
                ? "ON"
                : "OFF"
        );


        // ----------------------------------------------------
        // Calculated outputs.
        // ----------------------------------------------------

        Serial.print(" | OUT LAIL=");
        Serial.print(output.aileronLeft);

        Serial.print(" RAIL=");
        Serial.print(output.aileronRight);

        Serial.print(" ELE=");
        Serial.print(output.elevator);

        Serial.print(" ESC=");
        Serial.println(output.throttle);
    }
};


// ============================================================
// 13. SYSTEM OBJECTS
//
// Здесь создаётся конкретная конфигурация системы.
//
// В будущем именно этот участок будет похож на composition
// root приложения:
//   sensors
//   controllers
//   navigation
//   telemetry
//   GUI
//   etc.
// ============================================================

HardwareSerial IBusSerial(1);

IBusReceiver ibusReceiver(IBusSerial);

ControlMixer controlMixer;

ThrottleManager throttleManager;

ArmingManager armingManager;

FlightOutputs flightOutputs;

FlightController flightController(
    ibusReceiver,
    controlMixer,
    throttleManager,
    armingManager,
    flightOutputs
);

DebugLogger debugLogger(
    flightController
);


// ============================================================
// 14. SETUP
//
// setup() только запускает систему.
//
// Здесь не должно быть flight logic.
// ============================================================

void setup()
{
    // --------------------------------------------------------
    // Serial debug.
    // --------------------------------------------------------

    Serial.begin(115200);

    delay(1000);


    // --------------------------------------------------------
    // Startup message.
    // --------------------------------------------------------

    Serial.println();
    Serial.println("=================================");
    Serial.println(" AEROS-001 FLIGHT CONTROLLER");
    Serial.println(" ESP32-C3");
    Serial.println(" OOP ARCHITECTURE");
    Serial.println("=================================");
    Serial.println();


    // --------------------------------------------------------
    // Инициализация физических PWM outputs.
    // --------------------------------------------------------

    const bool outputsOK =
        flightOutputs.begin();


    Serial.print("Flight outputs: ");

    Serial.println(
        outputsOK
            ? "OK"
            : "FAILED"
    );


    // --------------------------------------------------------
    // Сразу после старта выставляем безопасные значения.
    // --------------------------------------------------------

    flightOutputs.setFailsafe();


    // --------------------------------------------------------
    // Инициализация flight controller.
    // --------------------------------------------------------

    flightController.begin();


    // --------------------------------------------------------
    // Информационный вывод.
    // --------------------------------------------------------

    Serial.println();
    Serial.println("iBUS input initialized.");
    Serial.println("115200 baud.");
    Serial.println("10 channels.");
    Serial.println("Throttle must be LOW.");
    Serial.println("Motor is DISARMED.");
    Serial.println();
}


// ============================================================
// 15. MAIN LOOP
//
// loop() намеренно максимально маленький.
//
// Это одна из главных целей новой архитектуры.
//
// В будущем сюда можно будет добавить:
//
//   sensors.update();
//   autopilot.update();
//   navigation.update();
//   telemetry.update();
//   gui.update();
//
// При этом отдельные системы останутся независимыми.
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // Основной flight controller.
    // --------------------------------------------------------

    flightController.update();


    // --------------------------------------------------------
    // Отдельная debug-подсистема.
    // --------------------------------------------------------

    debugLogger.update();


    // --------------------------------------------------------
    // Небольшая пауза.
    // --------------------------------------------------------

    delay(2);
}