#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <inc.h>

/*
 * 重要变量用 Matrix，中间变量要是需要时传 Array
 */

/*
 * Rays from camera, for every pixel, a line cross this pixel and origin of camera coordinate
 * 每个像素拍摄的路径设为一个向量，本函数产生一个矩形平面，为每个像素生成这样一个向量，每个向量用两点定义，第一点为相机原点，
 * 第二点(x,y,z)为相机坐标系下给定z所求得的三维坐标，忽略镜头畸变
 * 参数：相机内参，相机分辨率，给定Z，camera_rays(相机坐标系下z=给定值的 rays from camera)
 * camrea_rays 格式: 3 x (img.h x img.w), 每列格式为(x,y,z)，像素保存顺序为从左往右，从上往下
 */
bool configCameraRay(const Matrix3f& cameraMatrix, const cv::Size& imageSize, float Z, MatrixXf& camera_rays);

/*
 * 计算相机光线与参考平面的交点，参考平面用点法式表达
 * 参数：参考平面点，参考平面法线，rays from camera，保存交点矩阵
 * refPlane 格式：3 x (img.h x img.w), 每列格式为(x,y,z)，像素保存顺序为从左往右，从上往下
 */
bool configRefPlane(const Eigen::Vector3f& plane_point, const Eigen::Vector3f& plane_normal, const MatrixXf& camera_rays, MatrixXf& refPlane);

/* four step phase shifting
 * params: images(size=16), result wraped phase map(size=4)
 */
bool phase_shifting(const std::vector<MatrixXf>& imgs, std::vector<MatrixXf>& wraped_ps_maps);


#endif // ALGORITHM_H
