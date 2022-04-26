#include "algorithm_PMD.h"
#include <opencv2/core/eigen.hpp>
#define _USE_MATH_DEFINES
#include <math.h>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>

bool save_matrix_as_img(const Eigen::MatrixXf& m, std::string path) {
    MatrixXf t = (m.array() - m.minCoeff()).matrix();
    t = t * 255 / t.maxCoeff();
    Mat img;
    cv::eigen2cv(t, img);
    cv::imwrite(path, img);

    return true;
}

bool configCameraRay(const Eigen::Matrix3f& cameraMatrix, const cv::Size& imageSize, float Z, MatrixXf& camera_rays) {
    const uint16_t width = imageSize.width;
    const uint16_t height = imageSize.height;

    // 初始化camera_rays,初始值为归一化后的像素坐标(第三行为1)
    camera_rays.resize(3, height * width);
    {
        // row 1
        VectorXf t1(width);
        for (uint16_t i = 0; i < width; ++i)
            t1(i) = i;
        for (uint16_t i = 0; i < height; ++i)
            camera_rays.block(0, i * width, 1, width) = t1.transpose();

        // row 2
        for (uint16_t i = 0; i < height; i++)
            camera_rays.block(1, i * width, 1, width) = VectorXf::Constant(width, i).transpose();

        // row 3
        camera_rays.row(2) = VectorXf::Constant(1, height * width, 1).transpose();
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
    Vector3f M(0, 0, 0);
    float t1 = (plane_point - M).transpose() * plane_normal;

    ArrayXXf t =  t1 / (plane_normal.transpose() * camera_rays).array();
    refPlane.resize(camera_rays.rows(), camera_rays.cols());
    for (int i = 0; i < camera_rays.rows(); ++i)
        refPlane.row(i) = camera_rays.row(i).array() * t.array();
    refPlane = refPlane.colwise() + M;

    return true;
}

bool configScreenPixelPos(const Screen& S, const Matrix3f& WST, MatrixXf& screen_pix_pos) {
    // 每个像素去中心位置，如第一个像素取(0.5,0.5)
    // row 1
    VectorXf t1(S.cols);
    for (uint16_t i = 0; i < S.cols; ++i)
        t1(0, i) = i + 0.5;
    for (uint16_t i = 0; i < S.rows; ++i)
        screen_pix_pos.block(0, i * S.cols, 1, S.cols) = t1.transpose();
    screen_pix_pos.row(0) = S.width / S.cols * screen_pix_pos.row(0);

    // row 2
    for (uint16_t i = 0; i < S.rows; ++i)
        screen_pix_pos.block(1, i * S.cols, 1, S.cols) = VectorXf::Constant(S.cols, (i + 0.5) * S.height / S.rows).transpose();

    // row 3
    screen_pix_pos.row(2) = VectorXf::Constant(1, screen_pix_pos.cols(), 0).transpose();

    // 从屏幕坐标系转到世界坐标系
    MatrixXf t2(4, screen_pix_pos.cols());
    t2.block(0, 0, 3, t2.cols()) = screen_pix_pos;
    t2.row(3) = VectorXf::Constant(1, t2.cols(), 1);
    screen_pix_pos = (WST * t2).block(0, 0, 3, t2.cols());
    return true;
}

bool phase_shifting(const std::vector<MatrixXf>& imgs, std::vector<MatrixXf>& wraped_ps_maps) {
    // atan2((I1=I3)/(I0-I2))
    // 循环四组，而非每组四张图片
    for (uint16_t i = 0; i < 4; i++) {
        wraped_ps_maps[i] = MatrixXf(imgs[0].rows(), imgs[0].cols());
        MatrixXf tem0, tem1;
        tem0 = imgs[i * 4 + 1] - imgs[i * 4 + 3];
        tem1 = imgs[i * 4] - imgs[i * 4 + 2];
        wraped_ps_maps[i] = tem0.binaryExpr(tem1, [](float a, float b) {
            return atan2(a, b);
        });
    }

    return true;
}

bool phase_unwrapping(vector<MatrixXf>& wraped_ps_maps, uint16_t period_width, uint16_t period_height, vector<MatrixXf>& unwrap_ps_maps) {
    // 由于制作条纹时相位减了pi，因此加上一个pi(参照低频条纹，不减pi相位是分段的)，来作为真实相位
    for (int i = 0; i < 2; ++i) {
        uint16_t period = i == 0 ? period_width : period_height;
        unwrap_ps_maps[i] = ((period * (wraped_ps_maps[i + 2].array() + M_PI).matrix() - (wraped_ps_maps[i].array() + M_PI).matrix()) /
                             (2 * M_PI)).array().round().matrix(); // 计算得到 K
        unwrap_ps_maps[i] = (wraped_ps_maps[i].array() + M_PI).matrix() + 2 * M_PI * unwrap_ps_maps[i]; // phi + 2*pi*k
    }
    return true;
}

double computeReprojectionErrors(
    const vector<vector<cv::Point3f>>& objectPoints,
    const vector<vector<cv::Point2f>>& imagePoints,
    const vector<Mat>& rvecs, const vector<Mat>& tvecs,
    const Mat& cameraMatrix, const Mat& distCoeffs,
    vector<float>& perViewErrors) {
    vector<cv::Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for (i = 0; i < (int)objectPoints.size(); i++) {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
                      cameraMatrix, distCoeffs, imagePoints2);
        err = cv::norm(Mat(imagePoints[i]), Mat(imagePoints2), cv::NORM_L2);
        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err * err / n);
        totalErr += err * err;
        totalPoints += n;
    }

    return std::sqrt(totalErr / totalPoints);
}

bool aruco_analyze(const vector<Mat>& imgs_board, const ArucoBoard& board, const Mat& cameraMatrix, const Mat& distCoeffs, vector<Mat>& Rmats,
                   vector<cv::Vec3f>& Tvecs, vector<vector<int>>& Ids, vector<vector<vector<cv::Point2f>>>& corners,
                   vector<vector<vector<cv::Point2f>>>& rejectedCandidates, std::string output_dir) {
    for (uint16_t i = 0; i < imgs_board.size(); i++) {
        cv::Mat img = imgs_board[i].clone();

        auto parameters = cv::aruco::DetectorParameters::create();
        cv::aruco::detectMarkers(img, board.dictionary, corners[i], Ids[i], parameters, rejectedCandidates[i], cameraMatrix, distCoeffs);

        Mat output_img;
        cvtColor(imgs_board[i], output_img, cv::COLOR_GRAY2RGB);
        cv::Vec3f rvec;
        cv::aruco::drawDetectedMarkers(output_img, corners[i], Ids[i], cv::Scalar(0, 255, 0));

        // transforms points from the board coordinate system to the camera coordinate system
        cv::Ptr<cv::aruco::GridBoard> aruco_board = cv::aruco::GridBoard::create(board.markers_size.width, board.markers_size.height, board.markerLength,
                board.markerSeparation, board.dictionary, 0);
        int boardNum = cv::aruco::estimatePoseBoard(corners[i], Ids[i], aruco_board, cameraMatrix, distCoeffs, rvec, Tvecs[i]);

        // 检测到标记板
        if (boardNum > 0) {
            Rodrigues(rvec, Rmats[i]);
            cv::drawFrameAxes(output_img, cameraMatrix, distCoeffs, rvec, Tvecs[i], 0.2, 1);
            imwrite(output_dir + "/sys_calib_output/img_poseBoard_" + std::to_string(i) + ".bmp", output_img);
        } else
            return false;
    }

    return true;
}

bool system_calib(vector<Matrix3f>& CVR, vector<Vector3f>& CVTL, Matrix3f& CSR, VectorXf& CSTL_D, vector<Vector3f> n) {
    vector<Vector3f> m(3);  // m12,m23,m31
    for (uint16_t i = 0; i < CVR.size(); i++) {
        Eigen::EigenSolver<Matrix3f> es(CVR[i]*CVR[(i + 1) % 3].transpose(), true);
        auto eval = es.eigenvalues();

        // get eigenvector whos corresponfing eigenvalue is 1
        for (int j = 0; j < eval.rows(); j++) {
            std::complex<double> evalj = eval(j);
            if (fabs(1 - evalj.real()) <= 0.0001) {
                auto evec = es.eigenvectors();
                for (int t = 0; t < evec.rows(); t++) {
                    m[i](t) = evec(t, j).real();
                }
                break;
            }
        }
    }

    // sovling three normal vector(n)
    n[0] = m[0].cross(m[2]);
    n[1] = m[0].cross(m[1]);
    n[2] = m[1].cross(m[2]);
    for (int i = 0; i < 3; i++) {
        if (n[i](2) > 0)
            n[i] = -n[i];
        n[i].normalize();
    }

    // sovling CSR
    Matrix3f I3 = Matrix3f::Identity(3, 3);
    Matrix3f CSR0 = (I3 - 2 * n[0] * n[0].transpose()) * CVR[0];
    Matrix3f CSR1 = (I3 - 2 * n[1] * n[1].transpose()) * CVR[1];
    Matrix3f CSR2 = (I3 - 2 * n[2] * n[2].transpose()) * CVR[2];
    CSR = (CSR0 + CSR1 + CSR2) / 3;

    MatrixXf T_A(9, 6);
    MatrixXf T_B(9, 1);
    Vector3f Zero3 = Vector3f::Zero(3);
    T_A << (I3 - 2 * n[0]*n[0].transpose()), 2 * n[0], Zero3, Zero3,
        (I3 - 2 * n[1]*n[1].transpose()), Zero3, 2 * n[1], Zero3,
        (I3 - 2 * n[2]*n[2].transpose()), Zero3, Zero3, 2 * n[2];
    T_B << CVTL[0],
        CVTL[1],
        CVTL[2];
    CSTL_D = T_A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(T_B);

    return true;
}





















