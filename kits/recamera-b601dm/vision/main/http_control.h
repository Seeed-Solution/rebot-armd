// Modified from sscma-example-sg200x solutions/aruco-detector/main/http_control.{h,cpp}
// (commit 9af8118, Apache-2.0): namespace rv, routes served by VisionControl.
#ifndef REBOT_VISION_HTTP_CONTROL_H_
#define REBOT_VISION_HTTP_CONTROL_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "vision_control.h"

namespace rv {

/** True if a Content-Type header value is application/json (parameters allowed, case-insensitive). */
bool is_json_content_type(const std::string& value);

/**
 * Pre-routing guard. Returns 0 to continue, 415 for a POST without
 * Content-Type: application/json, 405 for any method other than GET/POST
 * (including OPTIONS: preflights are never approved, no CORS headers are sent).
 */
int precheck_request(const std::string& method, const std::string& content_type);

enum class ReadStatus { Ok, Timeout, Closed };

/**
 * Read one HTTP request (headers + Content-Length body, max 8 KiB) from a
 * socket. The whole read, not each recv, must finish within deadline_ms;
 * EINTR is retried. Fills header_end (npos if the header never completed)
 * and the parsed Content-Type / Content-Length headers.
 */
ReadStatus read_http_request(int fd, int deadline_ms, std::string& buf, size_t& header_end,
                             std::string& content_type, size_t& content_length, bool& has_content_length);

class HttpControlServer {
public:
    static constexpr int kMaxClients = 8;
    static constexpr int kRequestDeadlineMs = 3000;  // headers + body, total

    explicit HttpControlServer(VisionControl& ctl) : ctl_(ctl) {}
    ~HttpControlServer();

    bool start(int port);
    void stop();

private:
    struct ClientContext {
        int fd = -1;
        std::atomic<bool> done{false};
        std::thread thread;
    };

    void accept_loop();
    void handle_client(int fd);
    void clean_finished_clients();

    VisionControl& ctl_;
    int server_fd_ = -1;
    std::atomic<bool> running_{false};
    std::unique_ptr<std::thread> thread_;
    std::mutex clients_mu_;
    std::vector<std::shared_ptr<ClientContext>> clients_;
};

}  // namespace rv

#endif  // REBOT_VISION_HTTP_CONTROL_H_
