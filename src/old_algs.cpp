    if (world.findGapsAheadOf(x, left, right)) {
        m_cam.mark(Point(gapX, gapYUpper), Scalar(0,0,255));
        m_cam.mark(Point(gapX, gapYLower), Scalar(0,0,255));
        int gapHeight = (gapYLower - gapYUpper);
        int targetY = gapYUpper + (gapYLower - gapYUpper) / 2;
        int targetX = gapX;
        float dx = targetX - x;
        float dy = targetY - y;
        double angle = atan2(dy, dx);
        double tapInterval = 0.27;
        //cout << "angle: " << angle << endl;
        double angleDiff = abs(angle - m_previousAngle);
        if (angle < 0) { // below target
            tapInterval *= -angle / PI;
        } else { // above target
            if (movingDown) {
                double factor = angle / (PI * (1 - angleDiff / PI));
                cout << "factor: " << factor << endl;
                tapInterval /= factor;
            } else {
                tapInterval /= angle / PI;
            }
        }
        m_cam.drawLine(pos, Point(targetX, targetY), Scalar(255, 0, 0));
        tapInterval = pow(tapInterval, 0.65);
        if (m_pendingInPipeTap == false || (float(clock () - m_lastInPipeTapTime) /  CLOCKS_PER_SEC) > tapInterval) {
            m_lastInPipeTapTime = clock();
            m_pendingInPipeTap = true;
            m_arm.tap();
        }
    } else {
        if (m_pendingInPipeTap == false || (float(clock () - m_lastInPipeTapTime) /  CLOCKS_PER_SEC) > 0.25) {
            //m_pendingInPipeTap = false;
            //if (isAtGapHeight) {
            m_lastInPipeTapTime = clock();
            m_pendingInPipeTap = true;
            //}
            m_arm.tap();
        }
    }
        //bool isAtGapHeight = y < gapYLower && y > gapYUpper + gapHeight * 0.6;
        //cout << ((movingDown && m_pendingInPipeTap && (float(clock () - m_lastInPipeTapTime) /  CLOCKS_PER_SEC)) ? "time!" : "") << endl;
        //cout << (movingDown ? "moving down" : "moving up") << endl;
        // if (movingDown && y > gapYUpper - gapHeight * 0.0 ||
        //     y > gapYLower - gapHeight * 0.1 ||
        //movingDown && m_pendingInPipeTap && (float(clock () - m_lastInPipeTapTime) /  CLOCKS_PER_SEC) > 0.25
        // if (m_pendingInPipeTap == false || (float(clock () - m_lastInPipeTapTime) /  CLOCKS_PER_SEC) > 0.3) {
        //     //m_pendingInPipeTap = false;
        //     //if (isAtGapHeight) {
        //         m_lastInPipeTapTime = clock();
        //         m_pendingInPipeTap = true;
        //     //}
        //     m_arm.tap();
        // }

    // } else if (y > 200) {
    //     m_arm.tap();
    // }
