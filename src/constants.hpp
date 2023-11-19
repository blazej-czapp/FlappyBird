#pragma once

#include "units.hpp"

using namespace std::literals::chrono_literals; // propagate when including this header, not a big deal

/*
 * These should be all constants that control the simulation (e.g. physical constants) plus flight control
 * (e.g. safety margins). There are a few more in featureDetector.cpp - those are fairly stable and shouldn't
 * need too much tinkering.
 * Unit of length is the width of the score box (the one that shows when you crash), including the border
 */

static constexpr Coordinate BIRD_X_COORDINATE{0.425f};

// motion constants obtained from disassembling the game apk
static constexpr Speed HORIZONTAL_SPEED{{0.0006082}};

// tap just sets a new vertical speed
// values lifted and converted from disassembly of the game's APK
static constexpr Speed JUMP_SPEED{{-0.00144329}};
static constexpr Speed TERMINAL_VELOCITY{{0.002061855}};
static constexpr Acceleration GRAVITY{{0.00000474226}}; // position grows down, gravity is positive

// "grow" pipes by this much in all directions for collision detection; the idea is to discard candidate
// paths during search that pass too close to the obstacles, saving time evaluating them further (the
// expectation being that we'll find a path that doesn't need to cut it this fine)
// it's a remnant of early experiments when we accepted the first path that didn't crash, not the largest
// clearance one,  I've got a feeling this makes no real difference to performance
static constexpr Distance SAFETY_BUFFER{0.000};

static constexpr Distance GROUND_SAFETY_BUFFER{0.015};

static constexpr Distance BIRD_RADIUS{0.062f};

static constexpr const TimePoint::duration SIMULATION_TIME_QUANTUM{75};

// the point during captureFrame() at which the actual state of the underlying image is captured
// (accounting for memory transfer etc.), between 0.0 and 1.0
static constexpr double CAPTURE_POINT = 0.0;
static constexpr double WEBCAM_CAPTURE_POINT = 0.1;

// ----------- arm control ------------

// time between tap() is issued to the arm and the game registering the tap
// also - wait this long after issuing a tap() to the arm before lifting
// going much lower than this causes the arm to be lifted too quickly and the tablet doesn't register the tap
// strictly speaking, we could break it down further between the tablet registering the tap and the game
// hopefully we don't have to (another thing to calibrate)
static constexpr std::chrono::milliseconds PHYSICAL_ARM_TAP_DELAY = 70ms;
static constexpr std::chrono::milliseconds SIMULATED_ARM_TAP_DELAY = 30ms;

// wait at least this long before issuing the next tap()
// could probably be lower but we're getting too many taps right now, so limiting spam
static constexpr std::chrono::milliseconds PHYSICAL_ARM_LIFT_DELAY = 50ms;
static constexpr std::chrono::milliseconds SIMULATED_ARM_LIFT_DELAY = 30ms;
