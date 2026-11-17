#include "intrinsics.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "json.hpp"

namespace rv {

namespace {

using ojson = nlohmann::ordered_json;

bool fail(std::string& field, std::string& message, const std::string& f, const std::string& m) {
    field = f;
    message = m;
    return false;
}

std::string fmt(double v) {
    char b[32];
    std::snprintf(b, sizeof(b), "%g", v);
    return b;
}

}  // namespace

bool parse_intrinsics_put(const std::string& body, const Intrinsics& current, Intrinsics& out,
                          uint64_t& expected_revision, std::string& field, std::string& message) {
    const nlohmann::json j = nlohmann::json::parse(body, nullptr, /*allow_exceptions=*/false);
    if (j.is_discarded() || !j.is_object()) return fail(field, message, "", "body must be a JSON object");

    static const char* kKnown[] = {"fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3", "width", "height",
                                   "marker_size", "expected_revision",
                                   // read-only fields of the GET body, ignored
                                   "revision", "overridden_by_cli", "config_path", "api_version"};
    for (auto it = j.begin(); it != j.end(); ++it) {
        bool known = false;
        for (const char* k : kKnown) known = known || it.key() == k;
        if (!known) return fail(field, message, it.key(), "unknown field \"" + it.key() + "\"");
    }

    // a required finite number
    auto number = [&](const char* k, double& dst) {
        const auto it = j.find(k);
        if (it == j.end()) return fail(field, message, k, std::string("\"") + k + "\" is required");
        if (!it->is_number() || !std::isfinite(it->get<double>()))
            return fail(field, message, k, std::string("\"") + k + "\" must be a finite number");
        dst = it->get<double>();
        return true;
    };
    auto range = [&](const char* k, double v, double lo, double hi, bool open) {
        const bool ok = open ? (v > lo && v < hi) : (v >= lo && v <= hi);
        if (ok) return true;
        return fail(field, message, k,
                    std::string("\"") + k + "\" must be in " + (open ? "(" : "[") + fmt(lo) + ", " + fmt(hi) +
                        (open ? ")" : "]"));
    };

    Intrinsics r;
    double width = 0, height = 0;
    aruco::CameraModel& c = r.cam;
    if (!number("fx", c.fx) || !range("fx", c.fx, 100, 5000, false)) return false;
    if (!number("fy", c.fy) || !range("fy", c.fy, 100, 5000, false)) return false;
    if (!number("width", width)) return false;
    if (width != kIntrinsicsWidth) return fail(field, message, "width", "\"width\" must be 640");
    if (!number("height", height)) return false;
    if (height != kIntrinsicsHeight) return fail(field, message, "height", "\"height\" must be 360");
    if (!number("cx", c.cx) || !range("cx", c.cx, 0, width, true)) return false;
    if (!number("cy", c.cy) || !range("cy", c.cy, 0, height, true)) return false;
    if (!number("k1", c.k1) || !range("k1", c.k1, -10, 10, false)) return false;
    if (!number("k2", c.k2) || !range("k2", c.k2, -10, 10, false)) return false;
    if (!number("k3", c.k3) || !range("k3", c.k3, -10, 10, false)) return false;
    if (!number("p1", c.p1) || !range("p1", c.p1, -1, 1, false)) return false;
    if (!number("p2", c.p2) || !range("p2", c.p2, -1, 1, false)) return false;

    r.marker_size = current.marker_size;
    if (j.contains("marker_size")) {
        if (!number("marker_size", r.marker_size)) return false;
        if (!(r.marker_size > 0 && r.marker_size <= 1))
            return fail(field, message, "marker_size", "\"marker_size\" must be in (0, 1] m");
    }

    const auto er = j.find("expected_revision");
    if (er == j.end()) return fail(field, message, "expected_revision", "\"expected_revision\" is required");
    if (!er->is_number_unsigned())
        return fail(field, message, "expected_revision", "\"expected_revision\" must be an unsigned integer");
    expected_revision = er->get<uint64_t>();

    out = r;
    return true;
}

bool merge_intrinsics_config(const std::string& existing, const Intrinsics& in, std::string& out, std::string& error) {
    ojson j = ojson::object();
    bool blank = true;
    for (char ch : existing) blank = blank && std::isspace(static_cast<unsigned char>(ch));
    if (!blank) {
        j = ojson::parse(existing, nullptr, /*allow_exceptions=*/false);
        if (j.is_discarded() || !j.is_object()) {
            error = "config file is not a JSON object; not overwritten";
            return false;
        }
    }
    const aruco::CameraModel& c = in.cam;
    j["fx"] = c.fx;
    j["fy"] = c.fy;
    j["cx"] = c.cx;
    j["cy"] = c.cy;
    j["k1"] = c.k1;
    j["k2"] = c.k2;
    j["p1"] = c.p1;
    j["p2"] = c.p2;
    j["k3"] = c.k3;
    j["marker_size"] = in.marker_size;
    out = j.dump(2) + "\n";
    return true;
}

bool read_text_file(const std::string& path, std::string& out, bool& missing) {
    missing = false;
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        missing = errno == ENOENT;
        return false;
    }
    out.clear();
    char buf[4096];
    for (;;) {
        const ssize_t n = ::read(fd, buf, sizeof(buf));
        if (n < 0 && errno == EINTR) continue;
        if (n < 0) {
            const int e = errno;
            ::close(fd);
            errno = e;
            return false;
        }
        if (n == 0) break;
        out.append(buf, static_cast<size_t>(n));
    }
    ::close(fd);
    return true;
}

bool write_file_atomic(const std::string& path, const std::string& text, std::string& error) {
    const size_t slash = path.rfind('/');
    const std::string dir = slash == std::string::npos ? "." : (slash == 0 ? "/" : path.substr(0, slash));
    const std::string tmp = path + ".tmp." + std::to_string(static_cast<long>(::getpid()));
    auto sys_fail = [&](const char* what) {
        error = std::string(what) + " " + tmp + ": " + std::strerror(errno);
        return false;
    };

    const int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (fd < 0) return sys_fail("open");
    size_t done = 0;
    while (done < text.size()) {
        const ssize_t n = ::write(fd, text.data() + done, text.size() - done);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) {
            sys_fail("write");
            ::close(fd);
            ::unlink(tmp.c_str());
            return false;
        }
        done += static_cast<size_t>(n);
    }
    if (::fsync(fd) != 0) {
        sys_fail("fsync");
        ::close(fd);
        ::unlink(tmp.c_str());
        return false;
    }
    if (::close(fd) != 0) {
        sys_fail("close");
        ::unlink(tmp.c_str());
        return false;
    }
    if (::rename(tmp.c_str(), path.c_str()) != 0) {
        error = "rename " + tmp + " -> " + path + ": " + std::strerror(errno);
        ::unlink(tmp.c_str());
        return false;
    }
    const int dfd = ::open(dir.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (dfd < 0) {
        error = "open dir " + dir + ": " + std::strerror(errno);
        return false;
    }
    const int rc = ::fsync(dfd);
    const int e = errno;
    ::close(dfd);
    if (rc != 0) {
        error = "fsync dir " + dir + ": " + std::strerror(e);
        return false;
    }
    return true;
}

bool persist_intrinsics(const std::string& path, const Intrinsics& in, std::string& error) {
    std::string existing;
    bool missing = false;
    if (!read_text_file(path, existing, missing)) {
        if (!missing) {
            error = "read " + path + ": " + std::strerror(errno);
            return false;
        }
        existing.clear();
    }
    std::string text;
    if (!merge_intrinsics_config(existing, in, text, error)) return false;
    return write_file_atomic(path, text, error);
}

}  // namespace rv
