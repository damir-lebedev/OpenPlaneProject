#pragma once
#include <math.h>
#include <stdint.h>

// ============================================================
// Батарея коптера: напряжение, ток, израсходованные мА·ч и состояние —
// как «Power & Battery» в Betaflight (имена и смысл настроек — BF).
//
//   напряжение = мВ на пине × vbat_scale / (1000 × vbat_divider ×
//                vbat_multiplier) (110 / 10 / 1 — делитель 1:11);
//   ток, А     = (мВ на пине − ibata_offset) / (ibata_scale × 0.1 мВ/А)
//                (ibata_scale 400 — 40 мВ на ампер, у 4-в-1 BLS 100A — 100);
//   мА·ч       — интеграл тока.
// Фильтры — ФНЧ с частотой 10 / период Гц (период в 0.1 с, как в BF):
// vbat_display_lpf_period — показ и состояние, vbat_sag_lpf_period —
// компенсация просадки (контроллеру), ibat_lpf_period — ток.
//
// Батарея есть, если напряжение между vbat_detect_cell_voltage и полной
// банкой (1S) или больше двух vbat_detect_cell_voltage, — так USB (5 В на
// делителе) батареей не считается. Банок — из описания коптера
// (battery_cells) или force_battery_cell_count, а не угадываются по
// напряжению, как в BF при 0: у Li-ion 6S полностью севший пакет (17 В)
// не отличить от полного 4S Li-Po. Подключили полную (банка выше
// vbat_full_cell_voltage) — wasFullWhenPluggedIn().
//
// Состояние по напряжению (как в BF): OK -> WARNING, когда напряжение не
// выше «предупреждения − vbat_hysteresis» дольше vbat_duration_for_warning,
// WARNING -> CRITICAL — так же по минимуму и vbat_duration_for_critical;
// обратно — когда поднялось выше порога. По расходу (use_cbat_alerts, нужен
// датчик тока и ёмкость): осталось не больше cbat_alert_percent — WARNING,
// ноль — CRITICAL. Итог — худшее из двух; пищит (alertState) — только с
// use_vbat_alerts. vbat_cutoff_percent < 100: с первого CRITICAL газ за
// 10 с плавно ограничивается до этой доли (LVC Betaflight).
//
// Химия — пороги по умолчанию и «напряжение под нагрузкой» для модели
// тяги: Li-Po 3.75 В, Li-HV 3.85, Li-ion 3.5 (у Li-ion под током просадка
// больше, а разряжают его глубже — до 2.8–3.0 В).
//
// Без Arduino — тестируется на ПК (test_copter).
// ============================================================

namespace copter
{
    enum class BatteryChemistry : uint8_t
    {
        LIPO,
        LIHV,
        LIION,
    };

    struct CellVoltages
    {
        float min, warning, max, loaded, full;
    };

    // Пороги по умолчанию (BF для Li-Po: 3.30 / 3.50 / 4.30, полная 4.10).
    inline CellVoltages cellVoltages(BatteryChemistry c)
    {
        switch (c)
        {
            case BatteryChemistry::LIHV: return { 3.30f, 3.50f, 4.40f, 3.85f, 4.25f };
            case BatteryChemistry::LIION: return { 2.80f, 3.10f, 4.25f, 3.50f, 4.10f };
            case BatteryChemistry::LIPO:
            default: return { 3.30f, 3.50f, 4.30f, 3.75f, 4.10f };
        }
    }

    inline const char* chemistryName(BatteryChemistry c)
    {
        switch (c)
        {
            case BatteryChemistry::LIHV: return "Li-HV";
            case BatteryChemistry::LIION: return "Li-ion";
            case BatteryChemistry::LIPO:
            default: return "Li-Po";
        }
    }

    enum class BatteryState : uint8_t
    {
        NOT_PRESENT,
        OK,
        WARNING,
        CRITICAL,
    };

    // Как в status Betaflight.
    inline const char* batteryStateName(BatteryState s)
    {
        switch (s)
        {
            case BatteryState::OK: return "OK";
            case BatteryState::WARNING: return "WARNING";
            case BatteryState::CRITICAL: return "CRITICAL";
            case BatteryState::NOT_PRESENT:
            default: return "NOT PRESENT";
        }
    }

    // Откуда ток (current_meter) и напряжение (battery_meter) — порядок как в BF.
    // Работает ADC; прочие принимаются (diff из BF), но тока/напряжения не дают.
    // (не ADC/ESC: у F405 в CMSIS есть макрос ADC)
    enum class CurrentMeter : uint8_t
    {
        NONE,
        ADC_PIN,
        VIRTUAL,
        ESC_SENSOR,
        MSP_INPUT,
    };

    enum class VoltageMeter : uint8_t
    {
        NONE,
        ADC_PIN,
        ESC_SENSOR,
    };

    struct BatteryConfig
    {
        float vbatScale = 110.0f;      // BF vbat_scale
        float vbatDivider = 10.0f;     // BF vbat_divider
        float vbatMultiplier = 1.0f;   // BF vbat_multiplier
        bool voltageSensor = true;     // battery_meter = ADC
        float ibataScale = 400.0f;     // BF ibata_scale, 0.1 мВ/А
        float ibataOffsetMv = 0.0f;    // BF ibata_offset, мВ
        bool currentSensor = false;    // current_meter = ADC
        uint8_t cells = 4;             // battery_cells или force_battery_cell_count
        float capacityMah = 1300.0f;   // bat_capacity или battery_mah
        float minCellVolts = 3.30f;
        float warningCellVolts = 3.50f;
        float maxCellVolts = 4.30f;
        float fullCellVolts = 4.10f;   // vbat_full_cell_voltage
        float detectCellVolts = 3.00f; // vbat_detect_cell_voltage
        float hysteresisVolts = 0.01f; // vbat_hysteresis (на всю батарею, как в BF)
        float warningDelayS = 2.0f;    // vbat_duration_for_warning (у BF 0)
        float criticalDelayS = 2.0f;   // vbat_duration_for_critical (у BF 0)
        bool useVbatAlerts = true;     // use_vbat_alerts
        bool useCbatAlerts = false;    // use_cbat_alerts
        float cbatAlertPercent = 10.0f;
        float lvcPercent = 100.0f;     // vbat_cutoff_percent: 100 — выключено
        float displayLpfPeriod = 30.0f;   // x0.1 с
        float sagLpfPeriod = 2.0f;
        float ibatLpfPeriod = 10.0f;
    };

    class BatteryMonitor
    {
    public:
        static constexpr float LVC_RAMP_S = 10.0f;   // как в BF

        void configure(const BatteryConfig& c)
        {
            cfg = c;
            if (cfg.cells == 0) cfg.cells = 1;
            if (cfg.vbatDivider <= 0.0f) cfg.vbatDivider = 1.0f;
            if (cfg.vbatMultiplier <= 0.0f) cfg.vbatMultiplier = 1.0f;
        }

        const BatteryConfig& config() const { return cfg; }

        // Напряжение на пине (мВ) -> вольты батареи.
        float pinToVolts(float pinMv) const
        {
            return pinMv * cfg.vbatScale / (1000.0f * cfg.vbatDivider * cfg.vbatMultiplier);
        }

        // Каждый цикл (или реже): мВ на пинах АЦП, прошло dt секунд.
        void update(float vbatPinMv, float currentPinMv, float dt)
        {
            const float v = cfg.voltageSensor ? pinToVolts(vbatPinMv) : 0.0f;
            const float a = cfg.currentSensor && cfg.ibataScale != 0.0f
                                ? fmaxf(0.0f, (currentPinMv - cfg.ibataOffsetMv) * 10.0f / cfg.ibataScale)
                                : 0.0f;
            if (!started)
            {
                displayVolts = sagVoltsF = v;
                fastAmps = a;
                started = true;
            }
            // батарею только что подключили — фильтры сразу на её напряжение, а не
            // ползут снизу через «критично»
            if (batteryState == BatteryState::NOT_PRESENT && plausible(v))
            {
                displayVolts = sagVoltsF = v;
                voltageState = BatteryState::OK;
                consumption = BatteryState::OK;
                belowSince = 0.0f;
                lvcOn = false;
                lvcTimer = 0.0f;
                full = v >= cfg.cells * cfg.fullCellVolts;
            }
            displayVolts += (v - displayVolts) * gain(cfg.displayLpfPeriod, dt);
            sagVoltsF += (v - sagVoltsF) * gain(cfg.sagLpfPeriod, dt);
            fastAmps += (a - fastAmps) * gain(cfg.ibatLpfPeriod, dt);
            drawnMah += a * dt * (1000.0f / 3600.0f);
            updateState(dt);
        }

        float volts() const { return displayVolts; }               // показ (vbat_display_lpf_period)
        float sagVolts() const { return sagVoltsF; }               // просадка под газом (vbat_sag_lpf_period)
        float cellVolts() const { return displayVolts / cfg.cells; }
        float averageCellVolts() const { return displayVolts / cfg.cells; }
        float amps() const { return fastAmps; }
        float mahDrawn() const { return drawnMah; }
        bool hasCurrent() const { return cfg.currentSensor; }
        uint8_t cells() const { return cfg.cells; }
        bool wasFullWhenPluggedIn() const { return batteryState != BatteryState::NOT_PRESENT && full; }

        // Худшее из «по напряжению» и «по расходу».
        BatteryState state() const
        {
            if (batteryState == BatteryState::NOT_PRESENT) return BatteryState::NOT_PRESENT;
            return voltageState > consumption ? voltageState : consumption;
        }
        BatteryState stateByVoltage() const { return batteryState == BatteryState::NOT_PRESENT ? batteryState : voltageState; }
        BatteryState stateByConsumption() const { return batteryState == BatteryState::NOT_PRESENT ? batteryState : consumption; }

        // Для пищалки и OSD: use_vbat_alerts выключен — молчит.
        BatteryState alertState() const
        {
            const BatteryState s = state();
            if (s == BatteryState::NOT_PRESENT) return s;
            return cfg.useVbatAlerts ? s : BatteryState::OK;
        }

        // vbat_cutoff_percent: множитель газа 0..1 (1 — без ограничения).
        float throttleLimit() const
        {
            if (!lvcOn || cfg.lvcPercent >= 100.0f) return 1.0f;
            const float k = fminf(1.0f, lvcTimer / LVC_RAMP_S);
            return 1.0f - (1.0f - fmaxf(0.0f, cfg.lvcPercent) / 100.0f) * k;
        }

        // Осталось, % — по мА·ч, если есть датчик тока и ёмкость, иначе по напряжению.
        float percentLeft() const
        {
            if (batteryState == BatteryState::NOT_PRESENT) return 0.0f;
            if (cfg.currentSensor && cfg.capacityMah > 0.0f)
            {
                return fmaxf(0.0f, 100.0f * (1.0f - drawnMah / cfg.capacityMah));
            }
            const float span = cfg.maxCellVolts - cfg.minCellVolts;
            return span > 0.0f ? fminf(100.0f, fmaxf(0.0f, 100.0f * (averageCellVolts() - cfg.minCellVolts) / span)) : 0.0f;
        }

        void resetDrawn() { drawnMah = 0.0f; }

        // Калибровка по мультиметру (как вкладка Power в конфигураторе BF):
        // какой vbat_scale / ibata_scale дал бы измеренное. 0 — мерить нечего.
        float vbatScaleFor(float measuredVolts) const
        {
            if (displayVolts < 1.0f || measuredVolts <= 0.0f) return 0.0f;
            return cfg.vbatScale * measuredVolts / displayVolts;
        }
        float ibataScaleFor(float measuredAmps) const
        {
            if (!cfg.currentSensor || fastAmps < 0.5f || measuredAmps <= 0.0f) return 0.0f;
            return cfg.ibataScale * fastAmps / measuredAmps;
        }

    private:
        // ФНЧ с частотой 10 / период Гц (как в BF), шаг dt.
        static float gain(float periodTenths, float dt)
        {
            if (periodTenths <= 0.0f) return 1.0f;
            const float rc = periodTenths / (10.0f * 2.0f * 3.14159265f);
            return dt / (rc + dt);
        }

        // Похоже ли на батарею (а не на USB или пустой пин).
        bool plausible(float v) const
        {
            const float d = cfg.detectCellVolts;
            return (v >= d && v <= cfg.maxCellVolts) || v > 2.0f * d;
        }

        void updateState(float dt)
        {
            if (!plausible(sagVoltsF))
            {
                batteryState = BatteryState::NOT_PRESENT;
                return;
            }
            batteryState = BatteryState::OK;
            const float v = displayVolts;
            const float warning = cfg.cells * cfg.warningCellVolts;
            const float critical = cfg.cells * cfg.minCellVolts;
            switch (voltageState)
            {
                case BatteryState::WARNING:
                    if (v <= critical - cfg.hysteresisVolts)
                    {
                        belowSince += dt;
                        if (belowSince >= cfg.criticalDelayS) setVoltageState(BatteryState::CRITICAL);
                    }
                    else
                    {
                        belowSince = 0.0f;
                        if (v > warning) voltageState = BatteryState::OK;
                    }
                    break;
                case BatteryState::CRITICAL:
                    if (v > critical) setVoltageState(BatteryState::WARNING);
                    break;
                case BatteryState::OK:
                default:
                    if (v <= warning - cfg.hysteresisVolts)
                    {
                        belowSince += dt;
                        if (belowSince >= cfg.warningDelayS) setVoltageState(BatteryState::WARNING);
                    }
                    else
                    {
                        belowSince = 0.0f;
                    }
                    break;
            }
            if (lvcOn) lvcTimer += dt;

            if (cfg.useCbatAlerts && cfg.currentSensor && cfg.capacityMah > 0.0f)
            {
                const float left = percentLeft();
                consumption = left <= 0.0f                    ? BatteryState::CRITICAL
                              : left <= cfg.cbatAlertPercent ? BatteryState::WARNING
                                                              : BatteryState::OK;
            }
            else
            {
                consumption = BatteryState::OK;
            }
        }

        void setVoltageState(BatteryState s)
        {
            voltageState = s;
            belowSince = 0.0f;
            if (s == BatteryState::CRITICAL) lvcOn = true;   // однажды — до новой батареи
        }

        BatteryConfig cfg;
        bool started = false;
        bool full = false;
        bool lvcOn = false;
        float lvcTimer = 0.0f;
        float belowSince = 0.0f;
        float displayVolts = 0.0f, sagVoltsF = 0.0f, fastAmps = 0.0f, drawnMah = 0.0f;
        BatteryState batteryState = BatteryState::NOT_PRESENT;
        BatteryState voltageState = BatteryState::OK;
        BatteryState consumption = BatteryState::OK;
    };
}
