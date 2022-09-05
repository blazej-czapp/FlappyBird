#pragma once

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <unistd.h>

class SimulatedArm : public Arm {
public:
    // taps at position x,y on the x11display
    SimulatedArm(int x, int y, Display* const x11display) : m_x(x), m_y(y), m_x11display(x11display) {}

    std::chrono::milliseconds liftDelay() const override {
        return SIMULATED_ARM_LIFT_DELAY;
    }

    std::chrono::milliseconds tapDelay() const override {
        return SIMULATED_ARM_TAP_DELAY;
    }

    void tap() override {

        Window root = DefaultRootWindow(m_x11display);
        XWarpPointer(m_x11display, None, root, 0, 0, 0, 0, m_x, m_y);

        XEvent event;

        memset(&event, 0x00, sizeof(event));

        event.type = ButtonPress;
        event.xbutton.button = Button1;
        event.xbutton.same_screen = True;

        XQueryPointer(m_x11display, RootWindow(m_x11display, DefaultScreen(m_x11display)), &event.xbutton.root, &event.xbutton.window, &event.xbutton.x_root, &event.xbutton.y_root, &event.xbutton.x, &event.xbutton.y, &event.xbutton.state);

        event.xbutton.subwindow = event.xbutton.window;

        while(event.xbutton.subwindow) {
            event.xbutton.window = event.xbutton.subwindow;

            XQueryPointer(m_x11display, event.xbutton.window, &event.xbutton.root, &event.xbutton.subwindow, &event.xbutton.x_root, &event.xbutton.y_root, &event.xbutton.x, &event.xbutton.y, &event.xbutton.state);
        }

        if (XSendEvent(m_x11display, PointerWindow, True, 0xfff, &event) == 0) fprintf(stderr, "Error\n");

        XFlush(m_x11display);

        usleep(std::chrono::duration_cast<std::chrono::microseconds>(tapDelay()).count());

        event.type = ButtonRelease;
        event.xbutton.state = 0x100;

        if (XSendEvent(m_x11display, PointerWindow, True, 0xfff, &event) == 0) fprintf(stderr, "Error\n");

        XFlush(m_x11display);
    }

private:
    int m_x, m_y;
    Display* const m_x11display;
};
