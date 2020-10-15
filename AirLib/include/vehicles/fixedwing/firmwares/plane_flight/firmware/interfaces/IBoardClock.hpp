#pragma once

#include <cstdint>

namespace plane_flight {

class IBoardClock {
public:
    virtual uint64_t micros() const = 0;
    virtual uint64_t millis() const = 0;
};

}