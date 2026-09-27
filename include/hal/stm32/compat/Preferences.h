#pragma once

// ============================================================
// Preferences для STM32: тот же API, что у NVS в ESP32 Arduino core,
// но хранилище — KeyValueStore в секторе флеша
// (hal/stm32/Stm32FlashStorage.h). Каталог compat/ подключён через
// -I только в env stm32h743 — там #include <Preferences.h> драйверов
// датчиков, автотриммера и настроек лога находит этот файл.
// ============================================================

#include "hal/stm32/Stm32FlashStorage.h"
#include "storage/KvPreferences.h"

class Preferences : public KvPreferences
{
public:
    Preferences();
};
