#pragma once

// http://libk8055.sourceforge.net/

class Arm {
public:
    Arm();
    void tap();
    void setBusy(bool val);
    void deactivate();

    static const int TAP_COOLDOWN = 200 * 1000; //ns

private:
    int init();
    bool m_isBusy;
};
