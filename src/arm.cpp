#include "arm.hpp"
#include <iostream>
#include <k8055.h>
#include <unistd.h>
#include <pthread.h>

// this was required in C++14, no longer so in C++17 but for educational value: Arm::TAP_COOLDOWN is ODR-used
// in driver.cpp so even though this is initialised in the class, a definition in the compilation unit was
// still required (although:
//     "(...) an object is odr-used if its value is read (unless it is a compile time constant)(...)
// so I'm not sure why this is actually ODR-used)
//constexpr std::chrono::milliseconds Arm::TAP_COOLDOWN;

Arm::Arm() : m_isBusy(false) {
    init();
}

constexpr char ADDR_0 = 1 << 0;
constexpr char ADDR_1 = 1 << 1;
constexpr char ADDR_2 = 1 << 2;
constexpr char ADDR_3 = 1 << 3;

int Arm::init() {
    /*
     * Returned value (https://bbs.archlinux.org/viewtopic.php?id=181967):
     *  - Bin 0000, Dec 0: No devices was found
     *  - Bin 0001, Dec 1: Card address 0 was found.
     *  - Bin 0010, Dec 2: Card address 1 was found.
     *  - Bin 0100, Dec 4: Card address 2 was found.
     *  - Bin 1000, Dec 8: Card address 3 was found.
     */
    const char dev = SearchDevices();
    int address = 0;
    switch (dev) {
        case 0:
            std::cout << "Could not find the k8055 board :(" << std::endl;
            return -2; // OpenDevice() seems to be returning -1 on failure
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
    }

    std::cout << "Found k8055 device at address: " << static_cast<int>(address) << std::endl;
    int status = OpenDevice(address);
    std::cout << "Open Device status: " << status << std::endl;

    m_initSuccess = status == 0;
    return status;
}

void* executeTap(void *arm) {
    SetAllDigital();
    usleep(120000);
    ClearAllDigital();
    usleep(60000);
    static_cast<Arm*>(arm)->setBusy(false);
    pthread_exit(NULL); // this is a noreturn
}

void Arm::tap() {
    if (m_isBusy || !m_initSuccess) {
        return;
    }
    setBusy(true);
    pthread_t thread;
    pthread_create(&thread, NULL, executeTap, this); // TODO can we reuse this thread?
}

void Arm::deactivate() {
    if (m_initSuccess) {
        // TODO join with worker thread
        ClearAllDigital();
        CloseDevice();
    }
}

void Arm::setBusy(bool val) {
    m_isBusy = val;
}
