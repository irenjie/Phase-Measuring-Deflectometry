#include "algorithm_PMD.h"

bool configCameraRay(const Eigen::Matrix3f& cameraMatrix, const cv::Size& imageSize, float Z, MatrixXf& camera_rays) {
    const uint16_t width = imageSize.width;
    const uint16_t height = imageSize.height;

    // 初始化camera_rays,初始值为归一化后的像素坐标(第三行为1)
    camera_rays.resize(3, height * width);
    {
        // row 1
        VectorXf t1(width);
        for (uint16_t i = 0; i < width; i++)
            t1(i) = i;
        for (uint16_t i = 0; i < height; i++)
            camera_rays.block(0, i * width, 1, width) = t1.transpose();

        // row 2
        for (uint16_t i = 0; i < height; i++)
            camera_rays.block(1, i * width, 1, width) = VectorXf::Constant(width, i).transpose();

        // row 3
        camera_rays.row(2) = MatrixXf::Constant(1, height * width, 1);
    }

    // 反归一化
    if (Z != 1.0)
        camera_rays *= Z;

    // 忽略镜头畸变，反相机内参
    camera_rays = cameraMatrix.inverse() * camera_rays;

    return true;
}

bool configRefPlane(const Eigen::Vector3f& plane_point, const Eigen::Vector3f& plane_normal, const Eigen::MatrixXf& camera_rays,
                    Eigen::MatrixXf& refPlane) {
    Eigen::Vector3f M(0, 0, 0);
    float t1 = (plane_point - M).transpose() * plane_normal;

    ArrayXXf t =  t1 / (plane_normal.transpose() * camera_rays).array();
    refPlane.resize(camera_rays.rows(), camera_rays.cols());
    for (int i = 0; i < camera_rays.rows(); ++i)
        refPlane.row(i) = camera_rays.row(i).array() * t.array();
    refPlane = refPlane.colwise() + M;

    return true;
}

bool phase_shifting(const std::vector<MatrixXf>& imgs, std::vector<MatrixXf>& wraped_ps_maps) {
    // atan2((I1=I3)/(I0-I2))
    for (uint16_t i = 0; i < 4; i++) {
        wraped_ps_maps[i] = MatrixXf(imgs[0].rows(), imgs[0].cols());
        MatrixXf tem0, tem1;
        tem0 = imgs[i * 4 + 1] - imgs[i * 4 + 3];
        tem1 = imgs[i * 4] - imgs[i * 4 + 2];
        wraped_ps_maps[i] = tem0.binaryExpr(tem1, [](float a, float b) {
            return atan2(a, b);
        });
    }

}















