#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <algorithm>
#include <iterator>

// ============================================================
// ЧЁРНЫЙ ЯЩИК: формат записи во флеш
//
// Раздел "blackbox" (partitions_blackbox.csv) — кольцо секторов по
// 4 КБ. Сектор начинается с заголовка (SectorHeader, 16 байт), дальше
// записи подряд. Запись не переходит через границу сектора: не
// влезла — хвост сектора остаётся стёртым (0xFF), запись уходит в
// следующий. Поэтому любой сектор читается сам по себе, и порча или
// пропажа одного сектора не сбивает разбор остальных.
//
// Полёт — непрерывная цепочка секторов с одним номером полёта, по
// порядку сквозного номера seq. Первый сектор полёта начинается со
// схемы и параметров (SCHEMA, INFO), дальше — данные.
//
// Запись во флеше: [тип u8][длина данных u8][данные][CRC-8]; данные
// всегда начинаются с t_us (u32, micros() в момент снимка). CRC-8
// (полином 0x07) — по типу, длине и данным: пропало питание, пока
// страница писалась (или запись легла на две страницы, а вторая не
// успела), — последние записи не сойдутся и отбросятся, а не попадут
// в разбор мусором. В очереди (BlackBoxRing) записи без CRC — его
// добавляет BlackBoxStorage::append(). Какие поля у какого типа —
// записано в самом логе: SCHEMA "16 IMU t_us:I gx:h/10 ...". Декодер
// (tools/blackbox.py) читает схему из лога, а не держит свою копию,
// поэтому старые логи читаются и после смены формата.
//
// Поле схемы — имя:тип[/делитель]. Тип — символ модуля struct в
// Python: I/i — u32/i32, H/h — u16/i16, B/b — u8/i8, f — float; s —
// текст до конца записи. Значение = сырое / делитель. Длинная схема
// режется на несколько записей SCHEMA: продолжение — "<id> + поля...".
// ============================================================

namespace BlackBoxFormat
{
    constexpr uint8_t VERSION = 2;   // 2 — CRC-8 у каждой записи

    constexpr uint32_t SECTOR_SIZE = 4096;
    constexpr uint32_t PAGE_SIZE = 256;                 // единица записи флеша
    constexpr uint32_t BLOCK_SIZE = 65536;              // единица быстрого стирания
    constexpr uint32_t SECTORS_PER_BLOCK = BLOCK_SIZE / SECTOR_SIZE;
    constexpr uint32_t MAGIC = 0x4242504F;              // "OPBB"

    constexpr size_t RECORD_HEADER = 2;                 // тип + длина
    constexpr size_t MAX_PAYLOAD = 255;
    constexpr size_t MAX_RECORD = RECORD_HEADER + MAX_PAYLOAD;
    constexpr size_t RECORD_CHECK = 1;                  // CRC-8 во флеше

    // Заголовок сектора. check — дополнение суммы первых 15 байт:
    // оборванная запись заголовка (питание пропало) не сойдётся.
    struct SectorHeader
    {
        uint32_t magic;
        uint32_t seq;       // сквозной номер сектора, растёт на 1
        uint32_t startMs;   // millis() в момент открытия сектора
        uint16_t flight;    // номер полёта
        uint8_t version;
        uint8_t check;
    };
    static_assert(sizeof(SectorHeader) == 16, "заголовок сектора — 16 байт");

    constexpr size_t SECTOR_HEADER_SIZE = sizeof(SectorHeader);
    constexpr size_t SECTOR_PAYLOAD = SECTOR_SIZE - SECTOR_HEADER_SIZE;

    inline uint8_t headerCheck(const SectorHeader& h)
    {
        uint8_t bytes[sizeof(SectorHeader)];
        memcpy(bytes, &h, sizeof(bytes));
        uint8_t sum = 0;
        for (size_t i = 0; i + 1 < sizeof(bytes); ++i) sum = static_cast<uint8_t>(sum + bytes[i]);
        return static_cast<uint8_t>(~sum);
    }

    inline SectorHeader makeHeader(uint32_t seq, uint16_t flight, uint32_t startMs)
    {
        SectorHeader h = {};
        h.magic = MAGIC;
        h.seq = seq;
        h.startMs = startMs;
        h.flight = flight;
        h.version = VERSION;
        h.check = headerCheck(h);
        return h;
    }

    inline bool isValid(const SectorHeader& h)
    {
        return h.magic == MAGIC && h.version == VERSION && h.check == headerCheck(h);
    }

    // --------------------------------------------------------
    // Типы записей
    // --------------------------------------------------------

    enum RecordType : uint8_t
    {
        REC_SCHEMA = 0x01,   // t_us, текст "<id> <имя> <поля>" (см. выше)
        REC_INFO   = 0x02,   // t_us, "ключ=значение" — параметры полёта
        REC_EVENT  = 0x03,   // t_us, текст события (ARM, режим, связь...)
        REC_END    = 0x04,   // t_us, причина остановки — последняя запись полёта
        REC_IMU    = 0x10,   // каждый такт: гироскоп, акселерометр, время работы цикла
        REC_CTRL   = 0x11,   // 100 Гц: углы, цели, стики, команды, выходы, ПИД, режим
        REC_RC     = 0x12,   // 50 Гц: каналы пульта
        REC_BARO   = 0x13,   // новый отсчёт барометра
        REC_MAG    = 0x14,   // новый отсчёт компаса (не чаще 50 Гц)
        REC_GPS    = 0x15,   // новое решение GPS
        REC_AIR    = 0x16,   // новый отсчёт трубки Пито
        REC_NAV    = 0x17,   // 10 Гц: дом, курс, навигация, автотриммер
        REC_SYS    = 0x18,   // 1 Гц: цикл, память, приёмник, сам ящик
        REC_POWER  = 0x19,   // 10 Гц: батарея и датчик тока (АЦП платы полётника)
        REC_ERASED = 0xFF,   // стёртый флеш: дальше в секторе записей нет
    };

    // Биты CtrlRecord::flags (по порядку — имена в INFO "bits.flags").
    namespace Flag
    {
        constexpr uint16_t ARMED       = 1u << 0;
        constexpr uint16_t RX_LOST     = 1u << 1;   // связи нет (любая причина)
        constexpr uint16_t RX_TIMEOUT  = 1u << 2;   // нет кадров iBUS
        constexpr uint16_t RX_FAILSAFE = 1u << 3;   // приёмник сообщил failsafe пульта
        constexpr uint16_t FS_ACTIVE   = 1u << 4;   // автопилот ведёт по потере связи
        constexpr uint16_t FS_GLIDE    = 1u << 5;
        constexpr uint16_t FS_RTH      = 1u << 6;
        constexpr uint16_t AUTO_THR    = 1u << 7;   // газом управляет автопилот
        constexpr uint16_t STALL       = 1u << 8;   // защита от сваливания
        constexpr uint16_t FENCE       = 1u << 9;   // геозабор нарушен
        constexpr uint16_t IMU_OK      = 1u << 10;
        constexpr uint16_t BARO_OK     = 1u << 11;
        constexpr uint16_t MAG_OK      = 1u << 12;
        constexpr uint16_t GPS_OK      = 1u << 13;
        constexpr uint16_t GPS_FIX     = 1u << 14;
        constexpr uint16_t AIR_OK      = 1u << 15;
        constexpr const char* NAMES =
            "armed rx_lost rx_timeout rx_failsafe fs_active fs_glide fs_rth auto_thr "
            "stall fence imu_ok baro_ok mag_ok gps_ok gps_fix air_ok";
    }

    // --------------------------------------------------------
    // Данные записей (без заголовка тип/длина)
    // --------------------------------------------------------

#pragma pack(push, 1)

    struct ImuRecord
    {
        uint32_t tUs;
        int16_t gyro[3];    // 0.1 °/с, авиационные знаки
        int16_t accel[3];   // мg, оси самолёта
        uint16_t workUs;    // сколько длилась работа этого такта цикла
    };

    struct CtrlRecord
    {
        uint32_t tUs;
        int16_t roll, pitch, yaw;                 // 0.01°
        int16_t wantRoll, wantPitch;              // 0.01°, цель автопилота
        int16_t stickRoll, stickPitch, stickYaw;  // мкс отклонения от стиков
        int16_t cmdRoll, cmdPitch, cmdYaw;        // мкс, итог автопилота
        int16_t flaps;                            // мкс
        uint16_t thrPilot;                        // мкс, газ со стика
        int16_t thrAuto;                          // 0.1 %, газ автопилота
        uint16_t out[7];                          // мкс: AIL-L AIL-R ELE RUD ESC AUX1 AUX2
        int16_t pidRoll[3];                       // 0.1 мкс: P I D
        int16_t pidPitch[3];
        uint8_t mode;
        uint16_t flags;                           // Flag::*
        uint16_t features;                        // бит = Feature
    };

    struct RcRecord
    {
        uint32_t tUs;
        uint16_t ch[10];
        uint8_t status;    // бит 0 — нет кадров, бит 1 — failsafe пульта
        uint16_t good;     // счётчики кадров iBUS (младшие 16 бит)
        uint16_t bad;
    };

    struct BaroRecord
    {
        uint32_t tUs;
        float pressurePa;
        int16_t tempC;      // 0.01 °C
        float altM;         // над точкой включения
        int16_t vzCms;      // см/с
        int16_t targetDm;   // цель высоты автопилота, дм
    };

    struct MagRecord
    {
        uint32_t tUs;
        int16_t mag[3];     // 0.1 мкТл
        uint16_t heading;   // 0.01°
    };

    struct GpsRecord
    {
        uint32_t tUs;
        int32_t lat, lon;   // 1e-7°
        int32_t altMm;
        int16_t speedCms;
        uint16_t course;    // 0.01°
        uint8_t sats;
        uint8_t fix;
        uint16_t haccCm;
        uint16_t vaccCm;
    };

    struct AirRecord
    {
        uint32_t tUs;
        float dpPa;
        int16_t iasCms;
        int16_t tasCms;
        uint16_t rho;       // 1e-4 кг/м³
    };

    struct NavRecord
    {
        uint32_t tUs;
        uint8_t flags;      // бит 0 GPS годен, 1 дом есть, 2 геозабор, 3 сваливание
        float homeDistM;    // -1 — неизвестно
        uint16_t homeBearing, course, targetCourse;   // 0.01°
        int16_t speedCms;
        uint8_t courseSource;
        uint8_t launchState;
        uint8_t soaringState;
        int16_t trimRoll;   // 0.1 мкс
        int16_t trimPitch;
    };

    struct SysRecord
    {
        uint32_t tUs;
        uint16_t loopHz;
        uint16_t loopAvgUs;
        uint16_t loopMaxUs;     // худший такт за последнюю секунду
        uint32_t heapFree;
        uint32_t ibusGood;
        uint32_t ibusBad;
        int16_t imuTempC;       // 0.01 °C
        uint32_t ringUsed;      // байт в очереди на флеш
        uint32_t dropped;       // записей потеряно (очередь переполнена)
        uint16_t flashMaxUs;    // самая долгая запись страницы за секунду
        uint16_t freeKb;        // стёртого места впереди
    };

    struct PowerRecord
    {
        uint32_t tUs;
        uint16_t vbatMv;        // напряжение батареи (делитель пересчитан)
        uint16_t currentMv;     // выход датчика тока (делитель пересчитан)
    };

#pragma pack(pop)

    // --------------------------------------------------------
    // Схемы. Размер по строке схемы сверяется со struct на этапе
    // компиляции — поле, забытое в одном из мест, не соберётся.
    // --------------------------------------------------------

    struct Schema
    {
        uint8_t id;
        const char* name;
        const char* fields;
        size_t size;
    };

    constexpr size_t fieldSize(char type)
    {
        return (type == 'I' || type == 'i' || type == 'f') ? 4
             : (type == 'H' || type == 'h') ? 2
             : (type == 'B' || type == 'b') ? 1
             : 0;
    }

    // Сумма размеров полей: тип — символ сразу после ':'.
    constexpr size_t schemaSize(const char* fields, size_t total = 0)
    {
        return *fields == '\0' ? total
             : *fields == ':' ? schemaSize(fields + 2, total + fieldSize(fields[1]))
             : schemaSize(fields + 1, total);
    }

    constexpr const char* IMU_FIELDS =
        "t_us:I gx:h/10 gy:h/10 gz:h/10 ax:h/1000 ay:h/1000 az:h/1000 work_us:H";
    constexpr const char* CTRL_FIELDS =
        "t_us:I roll:h/100 pitch:h/100 yaw:h/100 want_roll:h/100 want_pitch:h/100 "
        "stick_roll:h stick_pitch:h stick_yaw:h cmd_roll:h cmd_pitch:h cmd_yaw:h flaps:h "
        "thr_pilot:H thr_auto:h/10 ail_l:H ail_r:H ele:H rud:H esc:H aux1:H aux2:H "
        "pid_roll_p:h/10 pid_roll_i:h/10 pid_roll_d:h/10 pid_pitch_p:h/10 pid_pitch_i:h/10 pid_pitch_d:h/10 "
        "mode:B flags:H features:H";
    constexpr const char* RC_FIELDS =
        "t_us:I ch1:H ch2:H ch3:H ch4:H ch5:H ch6:H ch7:H ch8:H ch9:H ch10:H status:B good:H bad:H";
    constexpr const char* BARO_FIELDS =
        "t_us:I pressure_pa:f temp_c:h/100 alt_m:f vz_ms:h/100 target_alt_m:h/10";
    constexpr const char* MAG_FIELDS =
        "t_us:I mx_ut:h/10 my_ut:h/10 mz_ut:h/10 heading:H/100";
    constexpr const char* GPS_FIELDS =
        "t_us:I lat:i/10000000 lon:i/10000000 alt_m:i/1000 speed_ms:h/100 course:H/100 "
        "sats:B fix:B hacc_m:H/100 vacc_m:H/100";
    constexpr const char* AIR_FIELDS =
        "t_us:I dp_pa:f ias_ms:h/100 tas_ms:h/100 rho:H/10000";
    constexpr const char* NAV_FIELDS =
        "t_us:I flags:B home_dist_m:f home_bearing:H/100 course:H/100 target_course:H/100 "
        "speed_ms:h/100 course_source:B launch_state:B soaring_state:B trim_roll:h/10 trim_pitch:h/10";
    constexpr const char* SYS_FIELDS =
        "t_us:I loop_hz:H loop_avg_us:H loop_max_us:H heap_free:I ibus_good:I ibus_bad:I "
        "imu_temp_c:h/100 ring_used:I dropped:I flash_max_us:H free_kb:H";

    static_assert(schemaSize(IMU_FIELDS) == sizeof(ImuRecord), "IMU: схема != struct");
    static_assert(schemaSize(CTRL_FIELDS) == sizeof(CtrlRecord), "CTRL: схема != struct");
    static_assert(schemaSize(RC_FIELDS) == sizeof(RcRecord), "RC: схема != struct");
    static_assert(schemaSize(BARO_FIELDS) == sizeof(BaroRecord), "BARO: схема != struct");
    static_assert(schemaSize(MAG_FIELDS) == sizeof(MagRecord), "MAG: схема != struct");
    static_assert(schemaSize(GPS_FIELDS) == sizeof(GpsRecord), "GPS: схема != struct");
    static_assert(schemaSize(AIR_FIELDS) == sizeof(AirRecord), "AIR: схема != struct");
    static_assert(schemaSize(NAV_FIELDS) == sizeof(NavRecord), "NAV: схема != struct");
    constexpr const char* POWER_FIELDS =
        "t_us:I vbat_v:H/1000 current_sensor_v:H/1000";

    static_assert(schemaSize(SYS_FIELDS) == sizeof(SysRecord), "SYS: схема != struct");
    static_assert(schemaSize(POWER_FIELDS) == sizeof(PowerRecord), "POWER: схема != struct");

    constexpr Schema SCHEMAS[] = {
        { REC_SCHEMA, "SCHEMA", "t_us:I text:s", 0 },
        { REC_INFO,   "INFO",   "t_us:I text:s", 0 },
        { REC_EVENT,  "EVENT",  "t_us:I text:s", 0 },
        { REC_END,    "END",    "t_us:I text:s", 0 },
        { REC_IMU,    "IMU",    IMU_FIELDS,  sizeof(ImuRecord) },
        { REC_CTRL,   "CTRL",   CTRL_FIELDS, sizeof(CtrlRecord) },
        { REC_RC,     "RC",     RC_FIELDS,   sizeof(RcRecord) },
        { REC_BARO,   "BARO",   BARO_FIELDS, sizeof(BaroRecord) },
        { REC_MAG,    "MAG",    MAG_FIELDS,  sizeof(MagRecord) },
        { REC_GPS,    "GPS",    GPS_FIELDS,  sizeof(GpsRecord) },
        { REC_AIR,    "AIR",    AIR_FIELDS,  sizeof(AirRecord) },
        { REC_NAV,    "NAV",    NAV_FIELDS,  sizeof(NavRecord) },
        { REC_SYS,    "SYS",    SYS_FIELDS,  sizeof(SysRecord) },
        { REC_POWER,  "POWER",  POWER_FIELDS, sizeof(PowerRecord) },
    };

    inline const Schema* findSchema(uint8_t type)
    {
        const Schema* found = std::find_if(std::begin(SCHEMAS), std::end(SCHEMAS),
                                           [type](const Schema& s) { return s.id == type; });
        return found == std::end(SCHEMAS) ? nullptr : found;
    }

    // Размер данных записи с фиксированной длиной; 0 — текстовая
    // (SCHEMA/INFO/EVENT/END) или неизвестный тип.
    inline size_t payloadSize(uint8_t type)
    {
        const Schema* s = findSchema(type);
        return s ? s->size : 0;
    }

    inline bool isKnownType(uint8_t type) { return findSchema(type) != nullptr; }

    // CRC-8/SMBUS (полином 0x07, начальное 0) — у каждой записи во флеше.
    inline uint8_t crc8(const uint8_t* data, size_t size)
    {
        uint8_t crc = 0;
        for (size_t i = 0; i < size; ++i)
        {
            crc ^= data[i];
            for (uint8_t bit = 0; bit < 8; ++bit)
            {
                crc = static_cast<uint8_t>((crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1);
            }
        }
        return crc;
    }

    // CRC-32 (как zlib.crc32 в Python) — для выгрузки по UART.
    inline uint32_t crc32(const uint8_t* data, size_t size, uint32_t crc = 0)
    {
        crc = ~crc;
        for (size_t i = 0; i < size; ++i)
        {
            crc ^= data[i];
            for (uint8_t bit = 0; bit < 8; ++bit)
            {
                crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
            }
        }
        return ~crc;
    }
}
