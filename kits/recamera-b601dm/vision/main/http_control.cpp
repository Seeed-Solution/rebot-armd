// Modified from sscma-example-sg200x solutions/aruco-detector/main/http_control.{h,cpp}
// (commit 9af8118, Apache-2.0): namespace rv, routes served by VisionControl.
#include "http_control.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cctype>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>

#if __has_include(<sscma.h>)
#include <sscma.h>
#endif
#ifndef MA_LOGI
#define MA_LOGI(tag, fmt, ...) fprintf(stdout, "[INFO][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define MA_LOGW(tag, fmt, ...) fprintf(stderr, "[WARN][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define MA_LOGE(tag, fmt, ...) fprintf(stderr, "[ERR][%s] " fmt "\n", tag, ##__VA_ARGS__)
#endif

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

#define TAG "rebot-vision-http"

namespace rv {

static constexpr size_t kMaxRequestBytes = 8192;

static const char* status_text(int code) {
    switch (code) {
        case 200: return "OK";
        case 400: return "Bad Request";
        case 404: return "Not Found";
        case 408: return "Request Timeout";
        case 405: return "Method Not Allowed";
        case 415: return "Unsupported Media Type";
        case 503: return "Service Unavailable";
        default: return "Error";
    }
}

bool is_json_content_type(const std::string& value) {
    size_t b = 0;
    while (b < value.size() && (value[b] == ' ' || value[b] == '\t')) ++b;
    size_t e = b;
    while (e < value.size() && value[e] != ';' && value[e] != ' ' && value[e] != '\t') ++e;
    std::string media = value.substr(b, e - b);
    for (char& c : media) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return media == "application/json";
}

int precheck_request(const std::string& method, const std::string& content_type) {
    if (method == "GET") return 0;
    if (method == "POST") return is_json_content_type(content_type) ? 0 : 415;
    return 405;  // OPTIONS and everything else: no CORS preflight is approved
}

static void send_reply(int fd, const HttpReply& r) {
    std::ostringstream ss;
    ss << "HTTP/1.1 " << r.status << " " << status_text(r.status) << "\r\n"
       << "Content-Type: application/json\r\n"
       << "Content-Length: " << r.body.size() << "\r\n"
       << "Cache-Control: no-store\r\n"
       << "Connection: close\r\n\r\n"
       << r.body;
    const std::string out = ss.str();
    size_t sent = 0;
    while (sent < out.size()) {
        const ssize_t n = ::send(fd, out.data() + sent, out.size() - sent, MSG_NOSIGNAL);
        if (n <= 0) break;
        sent += static_cast<size_t>(n);
    }
}

HttpControlServer::~HttpControlServer() {
    stop();
}

bool HttpControlServer::start(int port) {
    if (running_.load()) return true;
    ::signal(SIGPIPE, SIG_IGN);

    server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        MA_LOGE(TAG, "socket: %s", std::strerror(errno));
        return false;
    }
    int opt = 1;
    ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (::bind(server_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0 ||
        ::listen(server_fd_, 16) < 0) {
        MA_LOGE(TAG, "bind/listen 0.0.0.0:%d: %s", port, std::strerror(errno));
        ::close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    running_.store(true);
    thread_ = std::make_unique<std::thread>(&HttpControlServer::accept_loop, this);
    MA_LOGI(TAG, "Inference control on http://0.0.0.0:%d/api/inference", port);
    return true;
}

void HttpControlServer::clean_finished_clients() {
    std::vector<std::shared_ptr<ClientContext>> to_clean;
    {
        std::lock_guard<std::mutex> lk(clients_mu_);
        for (auto it = clients_.begin(); it != clients_.end(); ) {
            if ((*it)->done.load(std::memory_order_acquire)) {
                to_clean.push_back(std::move(*it));
                it = clients_.erase(it);
            } else {
                ++it;
            }
        }
    }
    for (auto& c : to_clean) {
        if (c->thread.joinable()) {
            c->thread.join();
        }
    }
}

void HttpControlServer::stop() {
    if (!running_.exchange(false)) return;

    ctl_.shutdown();

    if (server_fd_ >= 0) {
        ::shutdown(server_fd_, SHUT_RDWR);
    }

    std::vector<std::shared_ptr<ClientContext>> clients_copy;
    {
        std::lock_guard<std::mutex> lk(clients_mu_);
        for (auto& c : clients_) {
            if (c->fd >= 0) {
                ::shutdown(c->fd, SHUT_RDWR);
            }
        }
        clients_copy = std::move(clients_);
    }

    if (thread_ && thread_->joinable()) {
        thread_->join();
    }
    thread_.reset();

    if (server_fd_ >= 0) {
        ::close(server_fd_);
        server_fd_ = -1;
    }

    for (auto& c : clients_copy) {
        if (c->thread.joinable()) {
            c->thread.join();
        }
        if (c->fd >= 0) {
            ::close(c->fd);
            c->fd = -1;
        }
    }
}

void HttpControlServer::accept_loop() {
    while (running_.load()) {
        clean_finished_clients();
        const int sfd = server_fd_;
        if (sfd < 0) break;
        pollfd pfd = {sfd, POLLIN, 0};
        const int pr = ::poll(&pfd, 1, 200);
        if (pr <= 0 || !running_.load()) continue;

        sockaddr_in caddr;
        socklen_t clen = sizeof(caddr);
        const int cfd = ::accept(sfd, reinterpret_cast<sockaddr*>(&caddr), &clen);
        if (cfd < 0) continue;

        std::shared_ptr<ClientContext> ctx = std::make_shared<ClientContext>();
        ctx->fd = cfd;
        {
            std::lock_guard<std::mutex> lk(clients_mu_);
            if (!running_.load()) {
                ::close(cfd);
                continue;
            }
            if (clients_.size() >= kMaxClients) {
                send_reply(cfd, {503, "{\"error\":\"too many connections\"}"});
                ::close(cfd);
                continue;
            }
            ctx->thread = std::thread([this, ctx]() {
                handle_client(ctx->fd);
                {
                    std::lock_guard<std::mutex> lk(clients_mu_);
                    if (ctx->fd >= 0) {
                        ::close(ctx->fd);
                        ctx->fd = -1;
                    }
                }
                ctx->done.store(true, std::memory_order_release);
            });
            clients_.push_back(ctx);
        }
    }
}

static int64_t now_ms_steady() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

ReadStatus read_http_request(int fd, int deadline_ms, std::string& buf, size_t& header_end,
                             std::string& content_type, size_t& content_length, bool& has_content_length) {
    const int64_t deadline = now_ms_steady() + deadline_ms;
    char chunk[1024];
    header_end = std::string::npos;
    content_length = 0;
    has_content_length = false;
    content_type.clear();
    buf.clear();

    while (buf.size() < kMaxRequestBytes) {
        const int64_t remaining = deadline - now_ms_steady();
        if (remaining <= 0) return ReadStatus::Timeout;
        pollfd pfd = {fd, POLLIN, 0};
        const int pr = ::poll(&pfd, 1, static_cast<int>(remaining));
        if (pr < 0) {
            if (errno == EINTR) continue;
            return ReadStatus::Closed;
        }
        if (pr == 0) return ReadStatus::Timeout;
        const ssize_t n = ::recv(fd, chunk, sizeof(chunk), MSG_DONTWAIT);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
            return ReadStatus::Closed;
        }
        if (n == 0) break;
        buf.append(chunk, static_cast<size_t>(n));

        if (header_end == std::string::npos) {
            header_end = buf.find("\r\n\r\n");
            if (header_end != std::string::npos) {
                const std::string headers = buf.substr(0, header_end);
                std::istringstream hs(headers);
                std::string line;
                while (std::getline(hs, line)) {
                    if (!line.empty() && line.back() == '\r') line.pop_back();
                    const size_t colon = line.find(':');
                    if (colon == std::string::npos) continue;
                    std::string name = line.substr(0, colon);
                    for (char& c : name) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                    size_t vstart = colon + 1;
                    while (vstart < line.size() && (line[vstart] == ' ' || line[vstart] == '\t')) ++vstart;
                    if (name == "content-type") {
                        content_type = line.substr(vstart);
                    } else if (name == "content-length") {
                        content_length = std::strtoul(line.c_str() + vstart, nullptr, 10);
                        has_content_length = true;
                    }
                }
            }
        }
        if (header_end != std::string::npos) {
            const size_t body_have = buf.size() - (header_end + 4);
            if (!has_content_length || body_have >= content_length) return ReadStatus::Ok;
        }
    }
    return ReadStatus::Ok;
}

void HttpControlServer::handle_client(int fd) {
    std::string buf;
    size_t header_end = std::string::npos;
    size_t content_length = 0;
    bool has_content_length = false;
    std::string content_type;

    const ReadStatus rs = read_http_request(fd, kRequestDeadlineMs, buf, header_end, content_type,
                                            content_length, has_content_length);
    if (rs == ReadStatus::Timeout) {
        send_reply(fd, {408, "{\"error\":\"request timeout\"}"});
        return;
    }

    if (header_end == std::string::npos) {
        send_reply(fd, {400, "{\"error\":\"malformed HTTP request\"}"});
        return;
    }

    const std::string line1 = buf.substr(0, buf.find("\r\n"));
    const size_t s1 = line1.find(' ');
    const size_t s2 = s1 == std::string::npos ? std::string::npos : line1.find(' ', s1 + 1);
    if (s1 == std::string::npos || s2 == std::string::npos) {
        send_reply(fd, {400, "{\"error\":\"malformed request line\"}"});
        return;
    }
    const std::string method = line1.substr(0, s1);
    const std::string target = line1.substr(s1 + 1, s2 - s1 - 1);

    if (const int rc = precheck_request(method, content_type)) {
        send_reply(fd, {rc, rc == 415 ? "{\"error\":\"Content-Type must be application/json\"}"
                                      : "{\"error\":\"method not allowed\"}"});
        return;
    }

    const std::string body = buf.substr(header_end + 4, has_content_length ? content_length : std::string::npos);
    const HttpReply r = handle_control_request(ctl_, method, target, body, 0);
    send_reply(fd, r);
}

}  // namespace rv
