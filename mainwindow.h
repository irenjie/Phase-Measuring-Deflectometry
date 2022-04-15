#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>

#include "algorithm_PMD.h"


struct PMDcfg {
    cv::Size img_size;      // 相机图像分辨率
    Matrix3f cameraMatrix;  // 相机内参
    MatrixXf camera_rays;   // 相机光线
    Matrix4f CWT;           // 世界坐标系向量转到相机坐标系下的变换矩阵
    MatrixXf refPlane;      // 相机光线与参考平面的交点
};


QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

  public:
    MainWindow(QWidget* parent = nullptr);
    void test();
    ~MainWindow();

  private slots:
    void on_load_cfg_Button_clicked();

  private:
    Ui::MainWindow* ui;
    PMDcfg PMDcfg;
};



#endif // MAINWINDOW_H
