#include "algorithm_PMD.h"
#include <opencv2/core/eigen.hpp>
#define _USE_MATH_DEFINES
#include <math.h>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>

bool save_matrix_as_img(const MatrixXf& m, std::string path) {
    MatrixXf t = (m.array() - m.minCoeff()).matrix();
    t = t * 255 / t.maxCoeff();
    Mat img;
    cv::eigen2cv(t, img);
    cv::imwrite(path, img);

    return true;
}

bool save_matrix_as_txt(const MatrixXf& m, std::string path, bool add_index, int orientation) {
    std::ofstream fout(path);

    MatrixXf m2 = m;
    if (add_index) {
        if (orientation == 0) {
            VectorXf t(m2.cols());
            for (uint32_t i = 0; i < t.rows(); ++i)
                t(i) = i;
            m2.row(0) = t.transpose();
        } else {
            VectorXf t(m.rows());
            for (uint32_t i = 0; i < t.rows(); ++i)
                t(i) = i;
            m2.col(0) = t;
        }
    }
    fout << m2;
    fout.close();
    return true;
}

bool save_matrix_as_ply(const MatrixXf& m, std::string path) {
    std::ofstream fout(path);
    fout << "ply\n"
         "format ascii 1.0\n"
         "element vertex " << m.cols() << "\n"
         "property float32 x\n"
         "property float32 y\n"
         "property float32 z\n"
         "end_header\n";

    for (uint32_t j = 0; j < m.cols(); j++) {
        fout << m(0, j) << " " << m(1, j) << " " << m(2, j) << "\n";
    }
    fout.close();
    return true;
}

MatrixXf matrix_to_home(const Eigen::MatrixXf& origin) {
    MatrixXf m(origin.rows() + 1, origin.cols());
    m.block(0, 0, origin.rows(), origin.cols()) = origin;
    m.row(origin.rows()) = VectorXf::Constant(origin.cols(), 1).transpose();
    return m;
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
        camera_rays.row(2) = VectorXf::Constant(height * width, 1).transpose();
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
    if (M(0) != 0 || M(1) != 0 || M(2) != 0)
        refPlane = refPlane.colwise() + M;

    return true;
}

bool configScreenPixelPos(const Screen& S, const Matrix4f& WST, MatrixXf& screen_pix_pos) {
    screen_pix_pos.resize(3, S.rows * S.cols);

    // 每个像素去中心位置，如第一个像素取(0.5,0.5)
    // row 1
    VectorXf t1(S.cols);
    for (uint16_t i = 0; i < S.cols; ++i)
        t1(i) = i + 0.5;
    for (uint16_t i = 0; i < S.rows; ++i)
        screen_pix_pos.block(0, i * S.cols, 1, S.cols) = t1.transpose();
    screen_pix_pos.row(0) = S.width / S.cols * screen_pix_pos.row(0);

    // row 2
    for (uint16_t i = 0; i < S.rows; ++i)
        screen_pix_pos.block(1, i * S.cols, 1, S.cols) = VectorXf::Constant(S.cols, (i + 0.5) * S.height / S.rows).transpose();

    // row 3
    screen_pix_pos.row(2) = VectorXf::Constant(screen_pix_pos.cols(), 0).transpose();

    // 从屏幕坐标系转到世界坐标系
    MatrixXf t2 = matrix_to_home(screen_pix_pos);
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

bool phase_unwrapping(const vector<MatrixXf>& wraped_ps_maps, uint16_t period_width, uint16_t period_height, vector<MatrixXf>& unwrap_ps_maps) {
    // 由于制作条纹时相位减了pi，因此加上一个pi(参照低频条纹，不减pi相位是分段的)，来作为真实相位
    for (int i = 0; i < 2; ++i) {
        uint16_t period = i == 0 ? period_width : period_height;
        unwrap_ps_maps[i] = ((period * (wraped_ps_maps[i + 2].array() + M_PI).matrix() - (wraped_ps_maps[i].array() + M_PI).matrix()) /
                             (2 * M_PI)).array().round().matrix(); // 计算得到 K
        unwrap_ps_maps[i] = (wraped_ps_maps[i].array() + M_PI).matrix() + 2 * M_PI * unwrap_ps_maps[i]; // phi + 2*pi*k
    }
    return true;
}

bool screen_camera_phase_match(const vector<MatrixXf>& upm, const Screen& s, uint16_t pw, uint16_t ph, const MatrixXf& spp, MatrixXf& res) {
    uint32_t img_size = upm[0].cols() * upm[0].rows();
    MatrixXfR coor_x_f = upm[0] * s.cols / (2 * pw * M_PI);
    MatrixXfR coor_y_f = upm[1] * s.rows / (2 * ph * M_PI);
    coor_x_f.resize(img_size, 1);
    coor_y_f.resize(img_size, 1);
    VectorXUI coor_x = coor_x_f.array().round().matrix().cast<uint32_t>();
    VectorXUI coor_y = coor_y_f.array().round().matrix().cast<uint32_t>();

    res.resize(3, img_size);
    for (uint32_t i = 0; i < coor_x.rows(); ++i) {
        if (coor_x(i) >= upm[0].cols())
            coor_x(i) = upm[0].cols() - 1;
        if (coor_y(i) >= upm[0].rows())
            coor_y(i) = upm[0].rows() - 1;
        res.col(i) = spp.col(coor_y(i) * upm[0].cols() + coor_x(i));
    }

    return true;
}

bool slope_calculate(const Vector3f& camera_world, const MatrixXf& refPlane, const MatrixXf& screen_camera_phase_match_pos,
                     std::vector<MatrixXfR>& slope) {
    ArrayXXf dm2c, dm2s, M_C(3, refPlane.cols()), M_S;

    if (camera_world.cols() == 1) {
        M_C.row(0) = refPlane.row(0).array() - camera_world(0, 0);
        M_C.row(1) = refPlane.row(1).array() - camera_world(1, 0);
        M_C.row(2) = refPlane.row(2).array() - camera_world(2, 0);
    } else
        M_C = (refPlane - camera_world).array();
    M_S = (refPlane - screen_camera_phase_match_pos).array();

    dm2c = M_C.matrix().colwise().norm().array();
    dm2s = M_S.matrix().colwise().norm().array();

    ArrayXXf denominator = -(M_C.row(2) / dm2c + M_S.row(2) / dm2s);

    for (uint32_t i = 0; i < 2; ++i)
        slope[i] = ((M_C.row(i) / dm2c + M_S.row(i) / dm2s) / denominator).matrix();

    return true;
}

bool modal_reconstruction(const MatrixXf& sx, const MatrixXf& sy, MatrixXfR& Z, const std::vector<float>& rg, uint32_t terms) {
    // resize 斜率为2*M*N x 1
    MatrixXfR s(sx.rows() * 2, sx.cols());
    s.block(0, 0, sx.rows(), sx.cols()) = sx;
    s.block(sx.rows(), 0, sy.rows(), sy.cols()) = sy;
    s.resize(s.rows() * s.cols(), 1);

    // 多项式矩阵
    MatrixXfR slope_p(s.rows(), terms * terms);   // 斜率多项式矩阵
    MatrixXfR Z_p(s.rows() / 2, terms * terms); // 高度多项式矩阵
    double xa = rg[0], xb = rg[1], ya = rg[2], yb = rg[3];
    for (uint32_t i = 0; i < sx.rows(); ++i) {
        double y = (ya + yb - 2 * (yb - (yb - ya) / (sx.rows() - 1) * i)) / (ya - yb);
        for (uint32_t j = 0; j < sx.cols(); ++j) {
            double x = (xa + xb - 2 * ((xb - xa) / (sx.cols() - 1) * j + xa)) / (xa - xb);

            // 多项式有terms * terms 项，两层循环，先x后y, Dy_2 指 D_(n-2)
            float Dy_2 = 1, Dy_1 = y, dy_1 = 0;
            for (uint32_t n = 0; n < terms; ++n) {
                float Dy, dy;
                if (n == 0) {
                    Dy = 1;
                    dy = 0;
                } else if (n == 1) {
                    Dy = y;
                    dy = n * Dy_1 + y * dy_1;
                } else {
                    Dy = ((2 * n - 1) * y * Dy_1 - (n - 1) * Dy_2) / n;
                    dy = n * Dy_1 + y * dy_1;
                }

                float Dx_2 = 1, Dx_1 = x, dx_1 = 0;
                for (uint32_t m = 0; m < terms; ++m) {
                    float Dx, dx;
                    if (m == 0) {
                        Dx = 1;
                        dx = 0;
                    } else if (m == 1) {
                        Dx = x;
                        dx = m * Dx_1 + x * dx_1;
                    } else {
                        Dx = ((2 * m - 1) * x * Dx_1 - (m - 1) * Dx_2) / m;
                        dx = m * Dx_1 + x * dx_1;
                    }
                    slope_p(i * sx.cols() + j, n * terms + m) = dx * Dy;
                    slope_p(sx.cols()*sx.rows() + i * sx.cols() + j, n * terms + m) = Dx * dy;

                    Z_p(i * sx.cols() + j, n * terms + m) = Dx * Dy;

                    Dx_2 = Dx_1;
                    Dx_1 = Dx;
                    dx_1 = dx;
                }

                Dy_2 = Dy_1;
                Dy_1 = Dy;
                dy_1 = dy;
            }
        }
    }

    // 计算系数
    VectorXf C = slope_p.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(s);

    // 恢复高度
    Z = Z_p * C;
    Z.resize(sx.rows(), sx.cols());

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
        cv::aruco::drawDetectedMarkers(output_img, corners[i], Ids[i], cv::Scalar(0, 255, 0));

        // transforms points from the board coordinate system to the camera coordinate system
        cv::Ptr<cv::aruco::GridBoard> aruco_board = cv::aruco::GridBoard::create(board.markers_size.width, board.markers_size.height, board.markerLength,
                board.markerSeparation, board.dictionary, 0);
        cv::Vec3f rvec;
        int boardNum = cv::aruco::estimatePoseBoard(corners[i], Ids[i], aruco_board, cameraMatrix, distCoeffs, rvec, Tvecs[i]);

        // 检测到标记板
        if (boardNum > 0) {
            Rodrigues(rvec, Rmats[i]);
            cv::drawFrameAxes(output_img, cameraMatrix, distCoeffs, rvec, Tvecs[i], 0.04, 1);
            imwrite(output_dir + "/sys_calib_output/img_poseBoard_" + std::to_string(i) + ".bmp", output_img);
        } else
            return false;
    }

    return true;
}

bool system_calib(vector<Matrix3f>& CVR, vector<Vector3f>& CVTL, Matrix3f& CSR, VectorXf& CSTL_D, vector<Vector3f>& n) {
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
    Matrix3f CSR0 = (I3 - 2 * n[0] * n[0].transpose()).inverse() * CVR[0];
    Matrix3f CSR1 = (I3 - 2 * n[1] * n[1].transpose()).inverse() * CVR[1];
    Matrix3f CSR2 = (I3 - 2 * n[2] * n[2].transpose()).inverse() * CVR[2];
    CSR = (CSR0 + CSR1 + CSR2) / 3;

    MatrixXf T_A(9, 6);
    MatrixXf T_B(9, 1);
    Vector3f Zero3 = Vector3f::Zero(3);
    T_A << (I3 - 2 * n[0]*n[0].transpose()), -2 * n[0], Zero3, Zero3,
        (I3 - 2 * n[1]*n[1].transpose()), Zero3, -2 * n[1], Zero3,
        (I3 - 2 * n[2]*n[2].transpose()), Zero3, Zero3, -2 * n[2];
    T_B << CVTL[0],
        CVTL[1],
        CVTL[2];
    CSTL_D = T_A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(T_B);

    return true;
}

































