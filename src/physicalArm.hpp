#pragma once

#include "arm.hpp"
#include "constants.hpp"

#include <chrono>
#include <condition_variable>
#include <thread>

// http://libk8055.sourceforge.net/

using namespace std::chrono_literals;

class PhysicalArm : public Arm {
public:
    /// @param connect should the arm connect to the physical device, generally false for testing with recordings
    PhysicalArm(bool connect);
    void tap() override;

    ~PhysicalArm();

    std::chrono::milliseconds liftDelay() const override {
        return PHYSICAL_ARM_LIFT_DELAY;
    }

    std::chrono::milliseconds tapDelay() const override {
        return PHYSICAL_ARM_TAP_DELAY;
    }

private:
    void listenForTaps();

    /// is connected to k8055
    bool m_connected{false};

    /// This is what the worker thread checks to see if it should wake up and perform a tap - it is kept true for the
    /// whole duration of the tap so it doubles up as a 'busy' state flag.
    bool m_tapPending{false};

    std::mutex m_mutex;
    std::condition_variable m_handleEvent;

    std::thread m_workerThread;
};
