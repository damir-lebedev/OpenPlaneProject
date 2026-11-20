#pragma once

// ============================================================
// Нативная замена WebServer из Arduino core ESP32 2.0.x.
//
// Маршруты регистрируются как у настоящего сервера, а тест
// "присылает" запрос через request(метод, uri, тело) — вызывается
// тот же обработчик, ответ сохраняется в lastResponse().
// ============================================================

#include <algorithm>
#include <functional>
#include <string>
#include <vector>

#include "Arduino.h"

enum HTTPMethod
{
    HTTP_ANY,
    HTTP_GET,
    HTTP_HEAD,
    HTTP_POST,
    HTTP_PUT,
    HTTP_PATCH,
    HTTP_DELETE,
    HTTP_OPTIONS
};

class WebServer;

// Приём файла формой (как HTTPUpload у ESP32): обработчик загрузки зовут
// по частям — START, WRITE…, END, — затем обычный обработчик маршрута.
enum HTTPUploadStatus
{
    UPLOAD_FILE_START,
    UPLOAD_FILE_WRITE,
    UPLOAD_FILE_END,
    UPLOAD_FILE_ABORTED
};

struct HTTPUpload
{
    HTTPUploadStatus status = UPLOAD_FILE_START;
    String filename;
    String name;
    size_t totalSize = 0;
    size_t currentSize = 0;
    uint8_t buf[1436] = {};   // HTTP_UPLOAD_BUFLEN у ESP32
};

namespace fake
{
    // Все живые экземпляры WebServer: тест находит сервер, созданный
    // внутри WebDebugServer, не заглядывая в его приватные поля.
    inline std::vector<WebServer*>& webServers()
    {
        static std::vector<WebServer*> servers;
        return servers;
    }
}

class WebServer
{
public:
    typedef std::function<void(void)> THandlerFunction;

    struct Response
    {
        int code = 0;
        std::string contentType;
        std::string body;
    };

    explicit WebServer(int port = 80) : serverPort(port) { fake::webServers().push_back(this); }

    ~WebServer()
    {
        std::vector<WebServer*>& all = fake::webServers();
        for (size_t i = 0; i < all.size(); ++i)
        {
            if (all[i] == this)
            {
                all.erase(all.begin() + static_cast<long>(i));
                break;
            }
        }
    }

    WebServer(const WebServer&) = delete;
    WebServer& operator=(const WebServer&) = delete;

    void on(const String& uri, THandlerFunction handler) { on(uri, HTTP_ANY, handler); }
    void on(const String& uri, HTTPMethod method, THandlerFunction handler)
    {
        routes.push_back(Route{ uri.str(), method, handler, nullptr });
    }
    void on(const String& uri, HTTPMethod method, THandlerFunction handler, THandlerFunction uploadHandler)
    {
        routes.push_back(Route{ uri.str(), method, handler, uploadHandler });
    }
    HTTPUpload& upload() { return currentUpload; }
    void onNotFound(THandlerFunction handler) { notFound = handler; }

    void begin() { started = true; }
    void handleClient() { handleClientCalls++; }

    void send(int code, const char* contentType = nullptr, const String& content = String(""))
    {
        response.code = code;
        response.contentType = contentType ? contentType : "";
        response.body = content.str();
        sends++;
    }
    void send(int code, const String& contentType, const String& content) { send(code, contentType.c_str(), content); }
    void send_P(int code, PGM_P contentType, PGM_P content) { send(code, contentType, String(content)); }

    bool hasArg(const String& name) const { return name == "plain" && hasBody; }
    String arg(const String& name) const { return name == "plain" && hasBody ? String(body.c_str()) : String(); }

    // --- управление из тестов ---

    // Выполнить запрос; false — не нашлось ни маршрута, ни onNotFound.
    bool request(HTTPMethod method, const std::string& uri, const char* requestBody = nullptr)
    {
        response = Response();
        hasBody = requestBody != nullptr;
        body = requestBody ? requestBody : "";
        for (const Route& r : routes)
        {
            if (r.uri == uri && (r.method == HTTP_ANY || r.method == method))
            {
                r.handler();
                return true;
            }
        }
        if (notFound)
        {
            notFound();
            return true;
        }
        return false;
    }

    // POST формой: файл кусками по buf, abortAfter > 0 — клиент пропал после
    // стольких байт (ABORTED). false — маршрута нет.
    bool uploadFile(const std::string& uri, const std::string& fileName, const std::vector<uint8_t>& data, size_t abortAfter = 0)
    {
        response = Response();
        hasBody = false;
        for (const Route& r : routes)
        {
            if (r.uri != uri || r.method != HTTP_POST || !r.upload) continue;
            HTTPUpload& up = currentUpload;
            up = HTTPUpload();
            up.filename = String(fileName.c_str());
            up.name = String("firmware");
            up.status = UPLOAD_FILE_START;
            r.upload();
            size_t at = 0;
            while (at < data.size())
            {
                if (abortAfter && at >= abortAfter)
                {
                    up.status = UPLOAD_FILE_ABORTED;
                    r.upload();
                    return true;   // клиента нет — ответа тоже
                }
                const size_t n = std::min(sizeof(up.buf), data.size() - at);
                std::copy(data.begin() + static_cast<long>(at), data.begin() + static_cast<long>(at + n), up.buf);
                up.currentSize = n;
                up.status = UPLOAD_FILE_WRITE;
                r.upload();
                at += n;
                up.totalSize = at;
            }
            up.status = UPLOAD_FILE_END;
            up.currentSize = 0;
            r.upload();
            r.handler();
            return true;
        }
        return false;
    }

    const Response& lastResponse() const { return response; }
    int port() const { return serverPort; }
    bool isStarted() const { return started; }
    unsigned handleClientCount() const { return handleClientCalls; }
    size_t routeCount() const { return routes.size(); }

private:
    struct Route
    {
        std::string uri;
        HTTPMethod method;
        THandlerFunction handler;
        THandlerFunction upload;
    };

    int serverPort;
    std::vector<Route> routes;
    THandlerFunction notFound;
    bool started = false;
    unsigned handleClientCalls = 0;
    unsigned sends = 0;
    bool hasBody = false;
    std::string body;
    Response response;
    HTTPUpload currentUpload;
};
