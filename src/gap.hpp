#pragma once

#include "opencv2/core/types.hpp"
#include "units.hpp"

struct Gap {
    Position lowerLeft;
    Position lowerRight;
    Position upperLeft;
    Position upperRight;

    Gap& shiftHorizontally(const Distance &d) {
        lowerLeft.x.val += d.val;
        lowerRight.x.val += d.val;
        upperLeft.x.val += d.val;
        upperRight.x.val += d.val;
        return *this;
    }
};
