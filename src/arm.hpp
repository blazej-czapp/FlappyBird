#pragma once

#include <chrono>

class Arm {
public:
    virtual void tap() = 0;

    // time needed to lift the arm and get ready for the next tap
    virtual std::chrono::milliseconds liftDelay() const = 0;
    // time nevessary between a tap and the subsequent lift (needed for physical arm to properly register
    // the tap with the device)
    virtual std::chrono::milliseconds tapDelay() const = 0;
};
