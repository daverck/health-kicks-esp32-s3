#pragma once

#include <stdint.h>

extern uint32_t mockMillisValue;

inline uint32_t millis() {
    return mockMillisValue;
}

class MockSerial {
public:
    void println(const char*) {}

    template <typename... Args>
    void printf(const char*, Args...) {}
};

extern MockSerial Serial;
