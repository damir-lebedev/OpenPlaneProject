#pragma once

// ============================================================
// Связь SITL с внешним миром (src/sitl/SitlHost.cpp): TCP-сервер на
// один клиент (OpenPlane Studio или любой терминал), часы ПК для
// темпа «как в жизни», перезапуск процесса по reboot прошивки.
// Отдельный файл: заголовки Winsock не уживаются с фейками Arduino.
// ============================================================

#include <cstddef>
#include <cstdint>

namespace sitlhost
{
    // Слушать 127.0.0.1:port. false — порт занят.
    bool listen(uint16_t port);
    // Принять клиента, если ждёт; true — подключился новый (старого бросить).
    bool acceptPending();
    bool connected();
    // Без ожидания: сколько прочитано; 0 — нечего; клиент ушёл — 0 и connected() == false.
    int receive(uint8_t* buffer, int capacity);
    void send(const uint8_t* data, size_t size);
    void shutdown();

    // Монотонные часы ПК, мкс; сон — с точностью ~1 мс.
    uint64_t monotonicUs();
    void sleepUs(uint32_t us);

    // Запустить себя же с --child и ждать; вернуть код выхода ребёнка.
    int runChild(int argc, char** argv);
}
