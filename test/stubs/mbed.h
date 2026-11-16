#pragma once
/* Host-side stub of the mbed API surface used by src/main.cpp so the real
 * firmware logic can be unit-tested natively (PlatformIO native + Unity). */

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>

// ---- chrono literals (ms, as mbed uses them unqualified) ----
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuser-defined-literals"
struct mbed_duration {
    unsigned long long v;
    unsigned long long count() const { return v; }
};
inline constexpr unsigned long long operator""_ms(unsigned long long v) { return v; }
inline constexpr mbed_duration operator""ms(unsigned long long v) { return { v }; }
#pragma GCC diagnostic pop

// ---- pin names (only used as opaque tokens) ----
enum PinName { PA_0, PA_1, PA_5, PA_9, PB_6, PB_8, PB_9 };
enum PinMode { PullUp, PullNone };

// ---- test hooks: the tests install behaviour here ----
namespace mbed_stub {
    inline std::function<float()> analog_read_pa0 = [] { return 0.5f; };
    inline std::function<float()> analog_read_pa1 = [] { return 0.5f; };
    inline std::function<int()>  din_pb6 = [] { return 1; };  // cal start (active low)
    inline std::function<int()>  din_pa9 = [] { return 1; };  // cal next
}

class AnalogIn {
public:
    AnalogIn(PinName p) : pin(p) {}
    float read() {
        return pin == PA_0 ? mbed_stub::analog_read_pa0() : mbed_stub::analog_read_pa1();
    }
private:
    PinName pin;
};

class DigitalIn {
public:
    DigitalIn(PinName p, PinMode) : pin(p) {}
    int read() { return pin == PB_6 ? mbed_stub::din_pb6() : mbed_stub::din_pa9(); }
private:
    PinName pin;
};

class DigitalOut {
public:
    DigitalOut(PinName) {}
    void operator=(int v) { value = v; }
    int value = 0;
};

class Timer {
public:
    void start() {}
    mbed_duration elapsed_time() { return { ticks_ellapsed++ }; }
private:
    unsigned long long ticks_ellapsed = 0;
};

// ---- minimal threading stubs (not exercised by tests) ----
enum osPriority { osPriorityNormal, osPriorityRealtime };
class Thread {
public:
    Thread(osPriority) {}
    template <typename F> void start(F) {}
};
namespace ThisThread {
    inline void sleep_for(mbed_duration) {}
    inline void sleep_for(unsigned long long) {}
    inline void yield() {}
}

class I2CSlave {
public:
    enum Status { NoData = 0, WriteGeneral, WriteAddressed, ReadAddressed };
    I2CSlave(PinName, PinName) {}
    int receive() { return NoData; }
    void frequency(int) {}
    int address(int) { return 0; }
    int stop() { return 0; }
    int write(const char *, int) { return 0; }
    int read(char *, int) { return 0; }
};

#define __disable_irq() do {} while (0)
#define __enable_irq()  do {} while (0)
