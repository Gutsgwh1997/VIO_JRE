/**
 * @file IMU_sim.cpp
 * @brief 根据仿真得到的IMU观测值进行欧拉\中值积分
 * @author gwh
 * @version 1.0
 * @date 2019-11-27
 */
#include <fstream>
#include "../src/imu.h"
#include "../src/utilities.h"

int main()
{
    Param SimParams;
    IMU imugen(SimParams);

    std::vector<MotionData> imudata;
    std::vector<MotionData> imudata_noise;
    for (double t=SimParams.t_start; t<SimParams.t_end;)
    {
        //生成body系下的真实值
        MotionData imu = imugen.MotionModel(t);
        imudata.push_back(imu);

        //添加噪声
        imugen.addIMUnoise(imu);
        imudata_noise.push_back(imu);

        t+=SimParams.imu_timestep;
    }
    // 只有后六列是用的到的(角速度，加速度)
    save_Pose("../result/imu_true_value.txt",imudata);
    save_Pose("../result/imu_noise_value.txt",imudata_noise);
    imugen.init_Rwb_ = imudata[0].Rwb;
    imugen.init_twb_ = imudata[0].twb;
    imugen.init_velocity_ = imudata[0].imu_velocity;
    // 使用欧拉积分和中值积分求解轨迹
    imugen.testImu("../result/imu_true_value.txt","../result/imu_true_euler.txt",false);
    imugen.testImu("../result/imu_true_value.txt","../result/imu_true_median.txt",true);
    // 使用欧拉积分和中值积分求解带噪声的数据
    imugen.testImu("../result/imu_noise_value.txt","../result/imu_noise_euler.txt",false);
    imugen.testImu("../result/imu_noise_value.txt","../result/imu_noise_median.txt",true);

    return 0;
}
