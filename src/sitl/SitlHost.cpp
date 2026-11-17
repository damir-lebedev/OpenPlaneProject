// Сеть, часы и перезапуск для SITL (см. SitlHost.h). Только env:sitl.
#if defined(OPENPLANE_SITL)

#include "SitlHost.h"

#include <chrono>
#include <string>
#include <thread>
#include <vector>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <process.h>
typedef SOCKET Sock;
static const Sock BAD = INVALID_SOCKET;
static void closeSock(Sock s) { closesocket(s); }
static void nonBlocking(Sock s)
{
    u_long on = 1;
    ioctlsocket(s, FIONBIO, &on);
}
static bool wouldBlock() { return WSAGetLastError() == WSAEWOULDBLOCK; }
#else
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/wait.h>
#if defined(__linux__)
#include <signal.h>
#include <sys/prctl.h>
#endif
#include <unistd.h>
typedef int Sock;
static const Sock BAD = -1;
static void closeSock(Sock s) { close(s); }
static void nonBlocking(Sock s) { fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK); }
static bool wouldBlock() { return errno == EAGAIN || errno == EWOULDBLOCK; }
#endif

namespace
{
    Sock server = BAD;
    Sock client = BAD;

    void dropClient()
    {
        if (client != BAD) closeSock(client);
        client = BAD;
    }
}

namespace sitlhost
{
    bool listen(uint16_t port)
    {
#if defined(_WIN32)
        WSADATA wsa;
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return false;
        timeBeginPeriod(1);   // сон по 1 мс, а не по 15
#endif
        server = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (server == BAD) return false;
        sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (bind(server, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 || ::listen(server, 1) != 0)
        {
            closeSock(server);
            server = BAD;
            return false;
        }
        nonBlocking(server);
        return true;
    }

    bool acceptPending()
    {
        if (server == BAD) return false;
        Sock s = accept(server, nullptr, nullptr);
        if (s == BAD) return false;
        dropClient();
        client = s;
        nonBlocking(client);
        int one = 1;
        setsockopt(client, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof(one));
        return true;
    }

    bool connected() { return client != BAD; }

    int receive(uint8_t* buffer, int capacity)
    {
        if (client == BAD) return 0;
        const int n = static_cast<int>(recv(client, reinterpret_cast<char*>(buffer), capacity, 0));
        if (n > 0) return n;
        if (n < 0 && wouldBlock()) return 0;
        dropClient();   // 0 — закрыл, <0 — оборвался
        return 0;
    }

    void send(const uint8_t* data, size_t size)
    {
        // Клиент не успевает читать — не держим полёт: ждём до 50 мс, потом бросаем.
        const uint64_t deadline = monotonicUs() + 50000;
        while (client != BAD && size > 0)
        {
            const int n = static_cast<int>(::send(client, reinterpret_cast<const char*>(data), static_cast<int>(size), 0));
            if (n > 0)
            {
                data += n;
                size -= static_cast<size_t>(n);
                continue;
            }
            if (n < 0 && wouldBlock() && monotonicUs() < deadline)
            {
                sleepUs(500);
                continue;
            }
            dropClient();
        }
    }

    void shutdown()
    {
        dropClient();
        if (server != BAD) closeSock(server);
        server = BAD;
    }

    uint64_t monotonicUs()
    {
        using namespace std::chrono;
        return static_cast<uint64_t>(duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
    }

    void sleepUs(uint32_t us) { std::this_thread::sleep_for(std::chrono::microseconds(us)); }

    int runChild(int argc, char** argv)
    {
        std::vector<std::string> args;
        for (int i = 0; i < argc; ++i) args.push_back(argv[i]);
        args.push_back("--child");
#if defined(_WIN32)
        // Ребёнок — в задании Windows с «убить при закрытии»: убили родителя
        // (Studio остановила симулятор) — прошивка не остаётся висеть на порту.
        static HANDLE job = nullptr;
        if (!job)
        {
            job = CreateJobObjectW(nullptr, nullptr);
            JOBOBJECT_EXTENDED_LIMIT_INFORMATION info = {};
            info.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
            if (job) SetInformationJobObject(job, JobObjectExtendedLimitInformation, &info, sizeof(info));
            if (job) AssignProcessToJobObject(job, GetCurrentProcess());
        }
        // _spawnv склеивает аргументы через пробел — пути с пробелами в кавычки
        std::vector<std::string> quoted;
        for (const std::string& a : args) quoted.push_back(a.find(' ') != std::string::npos ? "\"" + a + "\"" : a);
        std::vector<const char*> av;
        for (const std::string& a : quoted) av.push_back(a.c_str());
        av.push_back(nullptr);
        const intptr_t code = _spawnv(_P_WAIT, argv[0], av.data());
        return code < 0 ? 1 : static_cast<int>(code);
#else
        std::vector<char*> av;
        for (std::string& a : args) av.push_back(&a[0]);
        av.push_back(nullptr);
        const pid_t pid = fork();
        if (pid == 0)
        {
#if defined(__linux__)
            prctl(PR_SET_PDEATHSIG, SIGKILL);   // родитель умер — и ребёнок
#endif
            execv(argv[0], av.data());
            _exit(1);
        }
        int status = 0;
        waitpid(pid, &status, 0);
        return WIFEXITED(status) ? WEXITSTATUS(status) : 1;
#endif
    }
}

#endif   // OPENPLANE_SITL
