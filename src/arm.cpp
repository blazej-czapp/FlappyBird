#include "arm.hpp"
#include <iostream>
#include <k8055.h>
#include <unistd.h>
#include <assert.h>

// this was required in C++14, no longer so in C++17 but for educational value: Arm::TAP_COOLDOWN is ODR-used
// in driver.cpp so even though this is initialised in the class, a definition in the compilation unit was
// still required (although:
//     "(...) an object is odr-used if its value is read (unless it is a compile time constant)(...)
// so I'm not sure why this is actually ODR-used)
//constexpr std::chrono::milliseconds Arm::TAP_COOLDOWN;

constexpr long ADDR_0 = 1u << 0u;
constexpr long ADDR_1 = 1u << 1u;
constexpr long ADDR_2 = 1u << 2u;
constexpr long ADDR_3 = 1u << 3u;

bool initArm() {
    /*
     * Returned value (https://bbs.archlinux.org/viewtopic.php?id=181967):
     *  - Bin 0000, Dec 0: No devices was found
     *  - Bin 0001, Dec 1: Card address 0 was found.
     *  - Bin 0010, Dec 2: Card address 1 was found.
     *  - Bin 0100, Dec 4: Card address 2 was found.
     *  - Bin 1000, Dec 8: Card address 3 was found.
     */
    const long dev = SearchDevices();
    int address = 0;
    switch (dev) {
        case 0:
            std::cout << "Could not find the k8055 board :(" << std::endl;
            return false;
        case ADDR_0:
            address = 0;
            break;
        case ADDR_1:
            address = 1;
            break;
        case ADDR_2:
            address = 2;
            break;
        case ADDR_3:
            address = 3;
            break;
        default:
            assert(false && "unexpected device address");
            return false;
    }

    std::cout << "Found k8055 device at address: " << static_cast<int>(address) << std::endl;
    int status = OpenDevice(address);
    std::cout << "k8055 OpenDevice status: " << status << std::endl;

    // OpenDevice() seems to be returning -1 on failure
    return status >= 0;
}

// time it takes for the arm to become ready (lift?) after tap. May need calibration.
constexpr static std::chrono::microseconds RAMP_DOWN{60000};

// Ramp up plus clear digital plus ramp down must be less than Arm::TAP_COOLDOWN or we'll either
// start dropping taps or have to wait for the arm to finish, whichever way we decide to implement tap().
// We don't know how long ClearAllDigital() takes so this is just a very crude sanity check.
static_assert(std::chrono::duration_cast<std::chrono::microseconds>(Arm::TAP_COOLDOWN) > Arm::TAP_DELAY + RAMP_DOWN);

void executeTap() {
    SetAllDigital();
    usleep(std::chrono::duration_cast<std::chrono::microseconds>(Arm::TAP_DELAY).count());
    ClearAllDigital();
    usleep(RAMP_DOWN.count());
}

// this runs under worker thread
void Arm::listenForTaps() {
    std::unique_lock<std::mutex> lock(m_mutex);
    while (m_connected) {
        m_handleEvent.wait(lock, [&]() { return m_tapPending || !m_connected; });

        if (m_tapPending) {
            executeTap();
            m_tapPending = false;
        } else {
            assert(!m_connected);
            return;
        }
    }
}

// Initialising the arm in the initializer list so that m_connected is correct right away.
// This simplifies listenForTaps() which knows right away if it should listen or bail out.
Arm::Arm(bool connect) : m_tapPending(false), m_connected(connect && initArm()), m_workerThread(&Arm::listenForTaps, this) {}

void Arm::tap() {
    if (!m_connected) {
        return;
    }

    if (m_tapPending) {
        std::cout << "Trying to tap a busy arm" << std::endl;
        return;
    }

    // This lock is belt and braces - in case m_tapPending has been unset in listenForTaps() but the loop
    // hasn't gone back to wait on condvar yet. If cooldown constants are right (and the rest of the code respects
    // them), this should never wait.
    // May be able to remove if it affects performance.
    {
        std::unique_lock<std::mutex> _(m_mutex);

        // We're setting m_tapPending under a mutex (not before) in case the worker thread wakes up spuriously, sees
        // pending is set and does a tap, while the main thread is then blocked for the whole duration.
        // Another possibility would be (if we set it after this scope):
        //  1. we acquire the lock
        //  2. worker wakes up spuriously, waits on the lock
        //  3. we release the lock, worker immediately acquires it, does the check before flags is set and decides
        //     it was a spurious wake up
        //  4. before worker goes to sleep, we set the flag and call notify which it misses, thus missing the tap
        //
        // I guess that's why you're supposed to modify shared variables under the same lock as the condvar:
        // (https://en.cppreference.com/w/cpp/thread/condition_variable)
        m_tapPending = true;
    }

    // If the worker thread wakes up now, fine, it will do the tap and notify will do nothing.
    // If it manages to finish the tap before we get to notify, that's also fine - the pending flag will be unset and
    // the notify will look like a spurious one and have no effect.

    m_handleEvent.notify_all(); // get the worker thread to do the tap
}

Arm::~Arm() {
    const bool wasConncted = m_connected;
    {
        // Once we're able to lock m_mutex, we know the worker thread is waiting on the condition variable
        // so we can set m_connected to false and notify. Without locking here, if the destructor
        // is called in the middle of a tap, we could set the flag and notify before worker comes back
        // to wait so it would never receive the notification and wait forever.
        std::unique_lock<std::mutex> _(m_mutex);
        // also, clear any tap request so the worker doesn't execute it and then hang on the condvar
        m_tapPending = false;
        m_connected = false;
    }

    m_handleEvent.notify_all();

    if (wasConncted) {
        ClearAllDigital();
        const bool success = CloseDevice() >= 0;
        std::cout << "Closing k8055 device: " << (success ? "SUCCESS" : "FAIL") << std::endl;
    }

    m_workerThread.join();
}
