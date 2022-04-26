#include "fullscreen.h"

FullScreen::FullScreen(QObject* parent) : QObject(parent) {

}

void FullScreen::initWindow() {
    cv::namedWindow("img", cv::WINDOW_NORMAL);
    cv::setWindowProperty("img", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

}

void FullScreen::closeWindow() {
    cv::destroyWindow("img");
}

void FullScreen::fs_one(const cv::Mat& img, uint16_t delay) {
    cv::imshow("img", img);
    cv::waitKey(delay);
    sendOK();
}
