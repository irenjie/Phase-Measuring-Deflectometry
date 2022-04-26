#ifndef FULLSCREEN_H
#define FULLSCREEN_H

#include <QObject>

#include <QObject>
#include <opencv2/opencv.hpp>


/*
 * 让图片全屏显示，调用 fs_one(img,delay)，将延迟delay时间后全屏显示一张图片,sendOK 信号表示显示完成
 */
class FullScreen : public QObject {
    Q_OBJECT
  public:
    explicit FullScreen(QObject* parent = nullptr);

  public slots:
    void initWindow();
    void closeWindow();

    /* 全屏显示一张图片,延迟发送显示完全信号，waitkey不会关掉显示窗口,延迟时间是应 >= 屏幕完全显示时间
     */
    void fs_one(const cv::Mat& img, uint16_t delay);

  signals:
    void sendOK();
};

#endif // FULLSCREEN_H
