// ============================================================
// Прошивка STM32H743 без SD-карты в слоте: самолёт должен загрузиться
// и летать как обычно, чёрный ящик — выключен и объясняет почему.
// Голая плата без датчиков — как на стенде.
//
// Запуск: pio test -e native-stm32 -f native_stm32/test_app_stm32_no_sd
// ============================================================

#include "../../../src/stm32/main.cpp"

#include <unity.h>

#include "helpers/TestSupport.h"

void setUp() {}
void tearDown() {}

namespace
{
    HardwareSerial& port(uint32_t rxPin)
    {
        HardwareSerial* p = fake::uartByRx(rxPin);
        TEST_ASSERT_NOT_NULL_MESSAGE(p, "UART с таким RX не создан");
        return *p;
    }

    void tick(const RcChannels& rc)
    {
        port(PE7).pushRx(ibusFrame(rc));
        fake::runTask(*fake::findTask("flight"), 1);   // задачи bbox нет: ящик выключен
    }
}

void test_boot_without_a_card_explains_and_leaves_the_box_off()
{
    setup();
    const std::string log = takeSerial();
    TEST_ASSERT_TRUE(contains(log, "SD-карта: нет ответа"));
    TEST_ASSERT_TRUE(contains(log, "BlackBox: нет места для записи"));
    TEST_ASSERT_EQUAL(BlackBox::State::Off, blackBox.getState());
    TEST_ASSERT_NOT_NULL(fake::findTask("flight"));
    TEST_ASSERT_NULL(fake::findTask("bbox"));   // писать некуда — задача не создаётся
}

void test_the_plane_still_flies_and_the_console_menu_does_not_break()
{
    // Голая плата без датчиков ARM не даст (нечему проверять), но стики
    // в сервы идут: ручной режим не зависит ни от датчиков, ни от карты.
    RcChannels rc;
    rc.set(Channels::SWC, 1000);
    rc.set(Channels::AILERON, 2000);
    for (int i = 0; i < 100; ++i) tick(rc);
    TEST_ASSERT_EQUAL(Config::AILERON_LEFT_REVERSED ? 1000 : 2000, fake::timerPulseUs(Config::PIN_AILERON_LEFT));
    TEST_ASSERT_EQUAL(BlackBox::State::Off, blackBox.getState());

    rc.set(Channels::AILERON, 1500);
    for (int i = 0; i < 50; ++i) tick(rc);
    takeSerial();
    Serial.pushRx("k");
    tick(rc);
    TEST_ASSERT_TRUE(contains(takeSerial(), "BlackBox: выключен"));

    Serial.pushRx(std::string("\x02") + "bb list\n");
    tick(rc);
    TEST_ASSERT_TRUE(contains(takeSerial(), "BB:STATE state=off"));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_boot_without_a_card_explains_and_leaves_the_box_off);
    RUN_TEST(test_the_plane_still_flies_and_the_console_menu_does_not_break);
    return UNITY_END();
}
