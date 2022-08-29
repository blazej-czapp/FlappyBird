#pragma once
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <unistd.h>

class SimulatedArm : public Arm {
public:
    SimulatedArm(int x, int y) : m_x(x), m_y(y) {}

    std::chrono::milliseconds liftDelay() const override {
        return 5ms;
    }

    std::chrono::milliseconds tapDelay() const override {
        return 20ms;
    }

    void tap() override {
        Display *display = XOpenDisplay(NULL);
        Window root = DefaultRootWindow(display);
        XWarpPointer(display, None, root, 0, 0, 0, 0, m_x, m_y);

        XEvent event;

        if(display == NULL)
        {
            fprintf(stderr, "Cannot initialize the display\n");
            exit(EXIT_FAILURE);
        }

        memset(&event, 0x00, sizeof(event));

        event.type = ButtonPress;
        event.xbutton.button = Button1;
        event.xbutton.same_screen = True;

        XQueryPointer(display, RootWindow(display, DefaultScreen(display)), &event.xbutton.root, &event.xbutton.window, &event.xbutton.x_root, &event.xbutton.y_root, &event.xbutton.x, &event.xbutton.y, &event.xbutton.state);

        event.xbutton.subwindow = event.xbutton.window;

        while(event.xbutton.subwindow)
        {
            event.xbutton.window = event.xbutton.subwindow;

            XQueryPointer(display, event.xbutton.window, &event.xbutton.root, &event.xbutton.subwindow, &event.xbutton.x_root, &event.xbutton.y_root, &event.xbutton.x, &event.xbutton.y, &event.xbutton.state);
        }

        if(XSendEvent(display, PointerWindow, True, 0xfff, &event) == 0) fprintf(stderr, "Error\n");

        XFlush(display);

        usleep(1000);

        event.type = ButtonRelease;
        event.xbutton.state = 0x100;

        if(XSendEvent(display, PointerWindow, True, 0xfff, &event) == 0) fprintf(stderr, "Error\n");

        XFlush(display);

        XCloseDisplay(display); //TODO needed?
    }

private:
    int m_x, m_y;
};
