/**
 * @file imu.h
 * @brief 此文件包含单帧IMU观测的结构体；
 *        采用预设轨迹的方式，IMU的测量值仿真(可以添加噪声)
 *        根据IMU测量值，采用欧拉积分得到轨迹；
 * @author gwh
 * @version 1.0
 * @date 2019-11-26
 */

#ifndef IMUSIMWITHPOINTLINE_IMU_H
#define IMUSIMWITHPOINTLINE_IMU_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <vector>

#include "param.h"

struct MotionData
{
    // 类中定义定长的Matrix或vector对象时，需要开辟内存，调用默认构造函数
    // 内存位数不对其．动态变量会动态分配内存.
    // 这里与VINS中的IMU观测量相同
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;
    Eigen::Matrix3d Rwb;
    Eigen::Vector3d twb;
    Eigen::Vector3d imu_acc;
    Eigen::Vector3d imu_gyro;

    Eigen::Vector3d imu_gyro_bias;
    Eigen::Vector3d imu_acc_bias;

    Eigen::Vector3d imu_velocity;
};

/**
 * @brief 欧拉角到旋转矩阵的变换，顺序z--y--x.(将bodyframe变换到inertia系下PPT42)
 *
 * @param eulerAngles 在I系下的欧拉角(roll, pitch, yaw),I系如何转到b系 
 *
 * @return 欧拉角对应的旋转矩阵(RIb)
 */
Eigen::Matrix3d euler2Rotation( Eigen::Vector3d  eulerAngles);


/**
 * @brief inertial frame下的欧拉角速度转换到body坐标系下的转换矩阵
 *
 * @param eulerAngles 在inertial系下的欧拉角
 *
 * @return 对应的旋转矩阵
 */
Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles);


class IMU
{
public:
    IMU(Param p);
    Param param_;
    Eigen::Vector3d gyro_bias_;
    Eigen::Vector3d acc_bias_;
    Eigen::Vector3d init_velocity_;
    Eigen::Vector3d init_twb_;
    Eigen::Matrix3d init_Rwb_;

    /**
     * @brief IMU仿真数据生成,在world系中的椭圆运动
     *
     * @param t 时间戳,按照增量的形式给(param中设定20s,与角速度配合使得yaw=2Pi)
     *
     * @return fake imu measurements,其中的Rwb,twb可以作为真实值
     */                                    
    MotionData MotionModel(double t);

    /**
     * @brief 主要给IMU仿真值imu_gyro与imu_acc添加噪声(bias与白噪声，方差在param中设定的)
     *
     * @param data 存储IMU观测值的结构体
     */
    void addIMUnoise(MotionData& data);

    /**
     * @brief 根据仿真获得的IMU的观测值，使用欧拉积分计算轨迹并保存
     *
     * @param src 载入IMU仿真获得的数据(文件路径)
     * @param dist 欧拉积分获得的轨迹文件保存的路径
     */
    void testImu(std::string src, std::string dist, bool median = false);    // imu数据进行积分，用来看imu轨迹

};

#endif //IMUSIMWITHPOINTLINE_IMU_H
