#include <string>
#include <iostream>

#include "arm.hpp"
#include "bird.hpp"
#include "camera.hpp"

int main(int argc, char** argv) {
    Camera cam;
    Arm arm;
    Bird bird{arm, cam};
    bool birdIsDriving = false;

    try {
        while (true) {
            cv::Point birdPos;
            cv::Mat world;
            cam.capture(birdPos, world);

            if (birdIsDriving) {
                World w{world, cam};
                bird.fly(w, birdPos);
            }

            const char key = cv::waitKey(30);
            if (key == 27) {
                std::cout << "esc key is pressed by user" << std::endl;
                break;
            } else if (key == 32 && !birdIsDriving) { // space
                std::cout << "space" << std::endl;
                arm.tap();
            } else if (key == 'a') {
                std::cout << "Switching to automatic" << std::endl;
                birdIsDriving = true;
            } else if (key == 'm') {
                std::cout << "Switching to manual" << std::endl;
                birdIsDriving = false;
            }
        }
    } catch (std::string& err) {
        std::cout << "Error: " << err << std::endl;
    }

    arm.deactivate();
    return 0;
}
