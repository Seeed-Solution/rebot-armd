#include <cstdio>

#include "capture_time.h"

static int g_fail = 0;
#define CHECK(c)                                                      \
    do {                                                              \
        if (!(c)) {                                                   \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);  \
            ++g_fail;                                                 \
        }                                                             \
    } while (0)

int main() {
    using aruco::capture_unix_ms;
    CHECK(capture_unix_ms(1726001234567ull, 5000, 4950) == 1726001234517ull);
    CHECK(capture_unix_ms(1726001234567ull, 5000, 5000) == 1726001234567ull);
    // 32-bit monotonic counter wrapped between grab and now
    CHECK(capture_unix_ms(1726001234567ull, 10, 0xFFFFFFF6u) == 1726001234567ull - 20);
    // frame stamp ahead of "now" (clock skew) must not produce a future/huge value from underflow
    CHECK(capture_unix_ms(1726001234567ull, 100, 101) <= 1726001234567ull);
    if (g_fail == 0) {
        std::printf("ALL CAPTURE TIME TESTS PASSED\n");
        return 0;
    }
    return 1;
}
