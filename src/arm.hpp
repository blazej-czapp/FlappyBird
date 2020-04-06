#pragma once

#include <chrono>
#include <condition_variable>
#include <thread>

// http://libk8055.sourceforge.net/

class Arm {
public:
    /// @param connect should the arm connect to the physical device, generally false for testing with recordings
    Arm(bool connect);
    void tap();

    ~Arm();

    constexpr static std::chrono::milliseconds TAP_COOLDOWN{200};
    constexpr static std::chrono::milliseconds TAP_DELAY{120}; // TODO rough guess, calibrate

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
