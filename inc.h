#ifndef PMDCFG_H
#define PMDCFG_H

/*
 * 该文件定义常用类型
 */

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include<Eigen/src/Core/util/Macros.h>

using std::endl;
using std::cout;
using std::vector;


typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXfR;
using Eigen::MatrixXf;
using Eigen::Matrix3f;
using Eigen::Matrix4f;
typedef Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic> MatrixXUI;

using Eigen::VectorXf;
using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector4f;
typedef Eigen::Matrix<uint32_t, Eigen::Dynamic, 1> VectorXUI;

using Eigen::ArrayXf;
using Eigen::ArrayXXf;

using cv::Mat;

#endif // PMDCFG_H
