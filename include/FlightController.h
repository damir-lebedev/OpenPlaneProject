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