#pragma once
#include <math.h>
#include <stdint.h>

// ============================================================
// Батарея коптера: напряжение, ток, израсходованные мА·ч и состояние —
// как «Power & Battery» в Betaflight.
//
//   напряжение = мВ на пине × vbat_scale / 10 (делитель платы, 110 — 1:11);
//   ток, А     = (мВ на пине − ibata_offset) / (ibata_scale × 0.1 мВ/А)
//                (ibata_scale 400 — 40 мВ на ампер, у 4-в-1 BLS 100A — 100);
//   мА·ч       — интеграл тока;
//   состояние  — по напряжению на банку, усреднённому за ~2 с (провал под
//                газом — не тревога): WARNING ниже vbat_warning_cell_voltage,
//                CRITICAL ниже vbat_min_cell_voltage; обратно — только с
//                запасом 0.05 В (иначе мигало бы на границе).
//
// Банок — из описания коптера (battery_cells), а не угадываются по
// напряжению: у Li-ion 6S полностью севший пакет (17 В) не отличить от
// полного 4S Li-Po.
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
        float min, warning, max, loaded;
    };

    // Пороги по умолчанию (BF для Li-Po: 3.30 / 3.50 / 4.30).
    inline CellVoltages cellVoltages(BatteryChemistry c)
    {
        switch (c)
        {
            case BatteryChemistry::LIHV: return { 3.30f, 3.50f, 4.40f, 3.85f };
            case BatteryChemistry::LIION: return { 2.80f, 3.10f, 4.25f, 3.50f };
            case BatteryChemistry::LIPO:
            default: return { 3.30f, 3.50f, 4.30f, 3.75f };
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

    struct BatteryConfig
    {
        float vbatScale = 110.0f;      // BF vbat_scale
        float ibataScale = 400.0f;     // BF ibata_scale, 0.1 мВ/А
        float ibataOffsetMv = 0.0f;    // BF ibata_offset, мВ
        bool currentSensor = false;    // есть ли датчик тока (current_meter = ADC)
        uint8_t cells = 4;
        float capacityMah = 1300.0f;
        float minCellVolts = 3.30f;
        float warningCellVolts = 3.50f;
        float maxCellVolts = 4.30f;
    };

    class BatteryMonitor
    {
    public:
        void configure(const BatteryConfig& c)
        {
            cfg = c;
            if (cfg.cells == 0) cfg.cells = 1;
        }

        const BatteryConfig& config() const { return cfg; }

        // Каждый цикл (или реже): мВ на пинах АЦП, прошло dt секунд.
        void update(float vbatPinMv, float currentPinMv, float dt)
        {
            const float v = vbatPinMv * 0.001f * cfg.vbatScale / 10.0f;
            const float a = cfg.currentSensor && cfg.ibataScale != 0.0f
                                ? fmaxf(0.0f, (currentPinMv - cfg.ibataOffsetMv) * 10.0f / cfg.ibataScale)
                                : 0.0f;
            if (!started)
            {
                fastVolts = slowVolts = v;
                fastAmps = a;
                started = true;
            }
            // батарею только что подключили — фильтры сразу на её напряжение, а не
            // ползут снизу через «критично»
            if (batteryState == BatteryState::NOT_PRESENT && v >= 0.8f * cfg.cells) fastVolts = slowVolts = v;
            fastVolts += (v - fastVolts) * clamp01(dt / 0.1f);
            slowVolts += (v - slowVolts) * clamp01(dt / 2.0f);
            fastAmps += (a - fastAmps) * clamp01(dt / 0.1f);
            drawnMah += fastAmps * dt * (1000.0f / 3600.0f);
            updateState();
        }

        float volts() const { return fastVolts; }
        float cellVolts() const { return fastVolts / cfg.cells; }
        float averageCellVolts() const { return slowVolts / cfg.cells; }
        float amps() const { return fastAmps; }
        float mahDrawn() const { return drawnMah; }
        bool hasCurrent() const { return cfg.currentSensor; }
        BatteryState state() const { return batteryState; }

        // Осталось, % — по мА·ч, если есть датчик тока, иначе по напряжению покоя.
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

    private:
        static float clamp01(float k) { return k > 1.0f ? 1.0f : k; }

        void updateState()
        {
            constexpr float HYSTERESIS = 0.05f;
            if (fastVolts < 0.8f * cfg.cells)   // меньше 0.8 В на банку — батареи нет (USB)
            {
                batteryState = BatteryState::NOT_PRESENT;
                return;
            }
            const float cell = slowVolts / cfg.cells;
            switch (batteryState)
            {
                case BatteryState::CRITICAL:
                    if (cell > cfg.minCellVolts + HYSTERESIS) batteryState = BatteryState::WARNING;
                    break;
                case BatteryState::WARNING:
                    if (cell < cfg.minCellVolts) batteryState = BatteryState::CRITICAL;
                    else if (cell > cfg.warningCellVolts + HYSTERESIS) batteryState = BatteryState::OK;
                    break;
                case BatteryState::NOT_PRESENT:
                case BatteryState::OK:
                default:
                    batteryState = cell < cfg.minCellVolts       ? BatteryState::CRITICAL
                                   : cell < cfg.warningCellVolts ? BatteryState::WARNING
                                                                 : BatteryState::OK;
                    break;
            }
        }

        BatteryConfig cfg;
        bool started = false;
        float fastVolts = 0.0f, slowVolts = 0.0f, fastAmps = 0.0f, drawnMah = 0.0f;
        BatteryState batteryState = BatteryState::NOT_PRESENT;
    };
}
