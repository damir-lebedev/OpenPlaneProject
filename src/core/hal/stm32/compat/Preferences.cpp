// Реализация hal/stm32/compat/Preferences.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "hal/stm32/compat/Preferences.h"


Preferences::Preferences()
: KvPreferences(Stm32FlashStorage::store())
{
}
