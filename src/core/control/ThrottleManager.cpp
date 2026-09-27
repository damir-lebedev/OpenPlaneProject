// Реализация control/ThrottleManager.h: вынесена из заголовка tools/split_headers.py
// (ветка feature/split-headers). Правки делайте в основной ветке и
// перегенерируйте — так две раскладки кода не расходятся.

#include "control/ThrottleManager.h"


auto ThrottleManager::update(
        const RcChannelState& rc,
        bool receiverFailsafe
    ) const -> uint16_t
{
    if (receiverFailsafe)
    {
        return Config::FAILSAFE_THROTTLE;
    }

    return RcInput::clamp(rc.get(Channels::THROTTLE));
}
