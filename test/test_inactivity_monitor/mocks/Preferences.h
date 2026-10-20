#pragma once

#include <stddef.h>
#include <stdint.h>

class Preferences {
public:
    bool begin(const char*, bool) { return true; }
    bool getBool(const char*, bool defaultValue) { return defaultValue; }
    uint16_t getUShort(const char*, uint16_t defaultValue) { return defaultValue; }
    size_t putBool(const char*, bool) { return 1; }
    size_t putUShort(const char*, uint16_t) { return 1; }
};
