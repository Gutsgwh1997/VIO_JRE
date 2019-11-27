//
// Created by hyj on 18-1-19.
//

#include "imu.h"
#include "utilities.h"

// euler2Rotation:   body frame to interitail frame
Eigen::Matrix3d euler2Rotation( Eigen::Vector3d  eulerAngles)
{
    double roll = eulerAngles(0);     //绕x轴转角
    double pitch = eulerAngles(1);    //绕y轴转角
    double yaw = eulerAngles(2);      //绕z轴转角

    double cr = cos(roll);  double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);
    double cy = cos(yaw);   double sy = sin(yaw);

    Eigen::Matrix3d RIb;
    RIb<<   cy*cp ,   cy*sp*sr - sy*cr,   sy*sr + cy* cr*sp,
            sy*cp,    cy *cr + sy*sr*sp,  sp*sy*cr - cy*sr,
            -sp,         cp*sr,           cp*cr;

    //PPT43页，将I系下的点变换到b系下．
    // Eigen::Matrix3d RbI;
    // RbI<< cp*cy,              cp*sy,             -sp,
    //       -cr*sy + sr*sp*cy,  cr*cy + sr*sp*sy,  sr*cp,
    //       sr*sy + cr*sp*cy,   -sr*cy + cr*sp*sy, cr*cp;

    return RIb;
}

Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);

    double cr = cos(roll);  double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);

    Eigen::Matrix3d R;
    R<<  1,   0,    -sp,
         0,   cr,   sr*cp,
         0,   -sr,  cr*cp;

    return R;
}


IMU::IMU(Param p): param_(p)   //初始化列表
{
    gyro_bias_ = Eigen::Vector3d::Zero();
    acc_bias_ = Eigen::Vector3d::Zero();
}

void IMU::addIMUnoise(MotionData& data)
{
    // C++11带来的新特性，random_device提供()运算符，返回min~max之间的一个数，可以理解为真随机数
    std::random_device rd;  
    std::default_random_engine generator_(rd());      // 引擎用来产生随机数，随机数引擎接受一个整形参数当作种子
    std::normal_distribution<double> noise(0.0, 1.0); // 产生标准正态分布的随机数,需要乘以标准差，得到对应正态分布(PPT26)

    Eigen::Vector3d noise_gyro(noise(generator_),noise(generator_),noise(generator_));     //加速度计的高斯白噪声
    //加速度计的标准差矩阵,只是为了矩阵运算,因为acc与gyro是三维向量
    Eigen::Matrix3d gyro_sqrt_cov = param_.gyro_noise_sigma * Eigen::Matrix3d::Identity(); 
    //！给仿真数据添加上了高斯白噪声与bias;
    //! 注意离散的标准差与连续的标准差相差根号△t
    data.imu_gyro = data.imu_gyro + gyro_sqrt_cov * noise_gyro / sqrt( param_.imu_timestep ) + gyro_bias_;

    Eigen::Vector3d noise_acc(noise(generator_),noise(generator_),noise(generator_));
    Eigen::Matrix3d acc_sqrt_cov = param_.acc_noise_sigma * Eigen::Matrix3d::Identity();
    data.imu_acc = data.imu_acc + acc_sqrt_cov * noise_acc / sqrt( param_.imu_timestep ) + acc_bias_;

    // gyro_bias update(PPT28)
    Eigen::Vector3d noise_gyro_bias(noise(generator_),noise(generator_),noise(generator_));
    gyro_bias_ += param_.gyro_bias_sigma * sqrt(param_.imu_timestep ) * noise_gyro_bias;
    data.imu_gyro_bias = gyro_bias_;

    // acc_bias update
    Eigen::Vector3d noise_acc_bias(noise(generator_),noise(generator_),noise(generator_));
    acc_bias_ += param_.acc_bias_sigma * sqrt(param_.imu_timestep ) * noise_acc_bias;
    data.imu_acc_bias = acc_bias_;

}

MotionData IMU::MotionModel(double t)
{
    MotionData data;
    // param
    float ellipse_x = 15;
    float ellipse_y = 20;
    float z = 1;          // z轴做sin运动
    float K1 = 10;        // z轴的正弦频率是x，y的k1倍
    float K = M_PI / 10;  // 20 * K = 2pi(角速度？) 由于我们采取的是时间是20s,
                          // 系数K控制yaw正好旋转一圈，运动一周
    /**
     * @name IMU轨迹生成
     * @{ IMU的测量值，是相对于惯性坐标系的加速度,角速度在载体坐标系的投影(PPT35) */
    /**   整体思路：设定IMU在world系下的轨迹，求导数得到速度与加速度(world系下);
     *    之后将角速度，加速度变换到body坐标系下                                @} */


    // translation
    // twb:  body frame in world frame
    // 椭圆的参数化表达，中心点在(5,5).
    Eigen::Vector3d position( ellipse_x * cos(K * t) + 5,
                              ellipse_y * sin(K * t) + 5,
                              z * sin(K1 * K * t) + 5);
    // 在world中的线速度
    Eigen::Vector3d dp( -K * ellipse_x * sin(K * t),
                         K * ellipse_y * cos(K * t),
                         z * K1 * K * cos(K1 * K * t) );  // position导数　in world frame

    double K2 = K*K;
    Eigen::Vector3d ddp( -K2 * ellipse_x * cos(K * t),
                         -K2 * ellipse_y * sin(K * t),
                         -z * K1 * K1 * K2 * sin(K1 * K * t) );  // position二阶导数,world中的加速度

    // Rotation
    double k_roll = 0.1;
    double k_pitch = 0.2;
    Eigen::Vector3d eulerAngles(k_roll * cos(t) , k_pitch * sin(t) , K*t );   // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
    Eigen::Vector3d eulerAnglesRates(-k_roll * sin(t) , k_pitch * cos(t) , K);      // euler angles 的导数in world frame

//    Eigen::Vector3d eulerAngles(0.0,0.0, K*t );   // roll ~ 0, pitch ~ 0, yaw ~ [0,2pi]
//    Eigen::Vector3d eulerAnglesRates(0.,0. , K);      // euler angles 的导数

    Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles);                      // body frame to world frame = body in world
    Eigen::Vector3d imu_gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;   //  euler rates trans to body gyro

    Eigen::Vector3d gn (0,0,-9.81);               //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)

    //IMU：　相对于惯性坐标系的测量值在载体坐标系下的投影***************
    Eigen::Vector3d imu_acc = Rwb.transpose() * ( ddp -  gn ); 

    // 模拟得到的IMU测量值
    data.imu_gyro = imu_gyro;
    data.imu_acc = imu_acc;
    data.Rwb = Rwb;              // 这些用来当做真实值
    data.twb = position;         // 在世界坐标系中的位置
    data.imu_velocity = dp;      // 在世界坐标系中的速度
    data.timestamp = t;
    return data;

}

//读取生成的imu数据并用imu动力学模型对数据进行计算，最后保存imu积分以后的轨迹，
//用来验证数据以及模型的有效性。
void IMU::testImu(std::string src, std::string dist, bool median )
{
    std::vector<MotionData>imudata;
    // 从文件载入IMU的观测数据，timestep;q;t;gyro;acc
    LoadPose(src,imudata);

    // 保存处理好(欧拉积分)后的数据
    std::ofstream save_points;
    save_points.open(dist);

    double dt = param_.imu_timestep;              // 以下设置初始位姿
    Eigen::Vector3d Pwb = init_twb_;              // position :    from  imu measurements
    Eigen::Quaterniond Qwb(init_Rwb_);            // quaterniond:  from imu measurements
    Eigen::Vector3d Vw = init_velocity_;          // velocity  :   from imu measurements
    Eigen::Vector3d gw(0,0,-9.81);                // ENU frame
    Eigen::Vector3d temp_a;
    Eigen::Vector3d theta;
    // 欧拉积分,i=0被用作初值，所以从１开始
    if (!median) {
        for (int i = 1; i < imudata.size(); ++i) {
            MotionData imupose = imudata[i];
            // delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
            Eigen::Quaterniond dq;
            Eigen::Vector3d dtheta_half = imupose.imu_gyro * dt / 2.0;
            dq.w() = 1;
            dq.x() = dtheta_half.x();
            dq.y() = dtheta_half.y();
            dq.z() = dtheta_half.z();

            /// imu 动力学模型 欧拉积分
            Eigen::Vector3d acc_w = Qwb * (imupose.imu_acc) +
                                    gw;  // a_b = Rbw(a_w - g_w) + bias + noise
            Qwb = Qwb * dq;
            Vw = Vw + acc_w * dt;
            Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
            //　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
            save_points<<imupose.timestamp<<" "
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<std::endl;
            
        }
    }
    // 中值积分
    else {
        for (int i = 1; i < imudata.size(); ++i) {
            // 中值积分
            MotionData imupose0 = imudata[i - 1];
            MotionData imupose1 = imudata[i];

            Eigen::Vector3d dtheta_half0 = imupose0.imu_gyro * dt / 2.0;
            Eigen::Vector3d dtheta_half1 = imupose1.imu_gyro * dt / 2.0;
            dtheta_half1 = 0.5 * dtheta_half1 + 0.5 * dtheta_half0;

            Eigen::Quaterniond dq_0_1;
            dq_0_1.w() = 1;
            dq_0_1.x() = dtheta_half1.x();
            dq_0_1.y() = dtheta_half1.y();
            dq_0_1.z() = dtheta_half1.z();
            Eigen::Quaterniond Qwb0;
            Eigen::Quaterniond Qwb1;
            Qwb0 = Qwb;
            Qwb1 = Qwb0 * dq_0_1;

            Eigen::Vector3d acc_w_0_1 = 0.5 * (Qwb0 * (imupose0.imu_acc) + gw) +
                                        0.5 * (Qwb1 * (imupose1.imu_acc) + gw);
            Qwb = Qwb1;
            Vw = Vw + acc_w_0_1 * dt;
            Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w_0_1;

            //　按着imu postion, imu quaternion , cam postion, cam quaternion
            //的格式存储，由于没有cam，所以imu存了两次
            save_points<<imupose1.timestamp<<" "
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<std::endl;
            
        }
    }
    std::cout<<"test　end"<<std::endl;

}
