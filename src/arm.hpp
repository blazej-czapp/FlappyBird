#pragma once

#include <chrono>

// http://libk8055.sourceforge.net/

class Arm {
public:
    Arm();
    void tap();
    void setBusy(bool val);
    void deactivate();

    constexpr static std::chrono::milliseconds TAP_COOLDOWN{200}; // * 1000; //TODO previously ns

private:
    int init();
    bool m_isBusy;
};
