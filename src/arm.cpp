#include "arm.hpp"
#include <iostream>
#include <k8055.h>
#include <iostream>
#include <unistd.h>
#include <pthread.h>
#include <cstdlib>

Arm::Arm() : m_isBusy(false) {
    init();
}

int Arm::init() {
    // char dev = SearchDevices();
    // cout << "Found Device: " << static_cast<int>(dev) << endl;
    // int status = OpenDevice(3);
    // cout << "Open Device: " << status << endl;
    //return status;
    return 1;
}

void* executeTap(void *arm) {
    // SetAllDigital();
    // usleep(120000);
    // ClearAllDigital();
    // usleep(60000);
    // static_cast<Arm*>(arm)->setBusy(false);
    // pthread_exit(NULL);
    // return NULL;
}

void Arm::tap() {
    // if (m_isBusy) {
    //     return;
    // }
    // setBusy(true);
    // pthread_t thread;
    // int rc = pthread_create(&thread, NULL, executeTap, this);

    // if (rc) {
    //     cout << "Error:unable to create thread," << rc << endl;
    //     exit(-1);
    // }
}

void Arm::deactivate() {
//    ClearAllDigital();
//    CloseDevice();
}

void Arm::setBusy(bool val) {
//    m_isBusy = val;
}
