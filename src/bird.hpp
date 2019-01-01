#pragma once

#include <deque>
#include <ctime>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "arm.hpp"
#include "camera.hpp"
#include "world.hpp"

// all times in seconds
const float TIME_ACROSS_SCREEN = 1.8f;
const float TIME_TO_FALL_WHOLE_SCREEN = 1.0f;
const float TIME_TO_RETURN_AFTER_JUMP = 0.2f; // to the same altitude
const float TIME_QUANTUM = 0.1f;
const float RELATIVE_JUMP_HEIGHT = 240.0f / 2000;
const float JUMP_DURATION = 0.3f;
const float MAINTAIN_ALTITUDE_TAP_INTERVAL = 0.4f;
//const float JUMP_SPEED = JUMP_HEIGHT / JUMP_DURATION;
//const int HORIZONTAL_MOVE_IN_QUANTUM = FRAME_WIDTH * (TIME_QUANTUM / TIME_ACROSS_SCREEN);

class Bird {
public:
    Bird(Arm& arm, Camera& cam);
    void fly(const World& world, const cv::Point& pos);

    static constexpr float RADIUS{0.05f}; // as proportion of screen width

private:
    Arm& m_arm;
    Camera& m_cam;
    clock_t m_lastTapped;

    struct State {
        State(State* parent, Camera& cam, cv::Point position, clock_t timeOfLastTap, bool rootTapped);

        bool canTap(clock_t now);
        State* tapped(clock_t now);
        State* notTapped(clock_t now);
        void childDied();
        bool hasCrashed(int noOfGaps, const Gap& left, const Gap& right);
        bool collidesWith(const Gap& gap);
        bool hitGround();
        void destroy();

        bool rootTapped;
    private:
        State *m_parent;
        int m_childrenCount;
        cv::Point m_position;
        clock_t m_lastTap;
        Camera& m_cam;
        unsigned m_horizontalMoveInQuantum;
        float m_jumpSpeed;
        float m_fallSpeed;
    };
};
