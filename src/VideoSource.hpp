#ifndef FLAPPYBIRD_VIDEOSOURCE_HPP
#define FLAPPYBIRD_VIDEOSOURCE_HPP

class VideoSource {
public:
    virtual const cv::Mat& captureFrame() = 0;
};

#endif //FLAPPYBIRD_VIDEOSOURCE_HPP
