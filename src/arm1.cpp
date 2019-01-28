//#include <iostream>
//#include <k8055.h>
//#include <iostream>
//#include <unistd.h>
//#include <pthread.h>
//#include <cstdlib>
//
//Arm::Arm() {
//    init();
//}
//
//int Arm::init() {
//    char dev = SearchDevices();
//    std::cout << "Found Device: " << dev << std::endl;
//    int status = OpenDevice(3);
//    std::cout << "Open Device: " << status << std::endl;
//    return status;
//}
//
//void* Arm::executeTap(void *_) {
//    SetAllDigital();
//    usleep(90000);
//    ClearAllDigital();
//    usleep(100000);
//    pthread_exit(NULL);
////    CloseDevice();
//    return NULL;
//}
//
//void Arm::tap() {
//    pthread_t thread;
//    int rc = pthread_create(&thread, NULL, executeTap, NULL);
//
//    if (rc) {
//        cout << "Error:unable to create thread," << rc << endl;
//        exit(-1);
//    }
//}
