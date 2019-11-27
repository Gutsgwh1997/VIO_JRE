#!/usr/bin/env python
# coding: utf-8

# ## 绘制IMU轨迹的脚本

# In[1]:


import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import os
#表示当前所处的文件夹上一级文件夹的绝对路径
filepath=os.path.abspath('..')+"/result"  


# In[2]:


# imu仿真得到的不加噪声的真值
position = []
quaterntions = []
timestamp = []
tx_index = 5

with open(filepath + '/imu_true_value.txt', 'r') as f:
    
    data = f.readlines()     # txt中所有字符串读入data
    for line in data:
        odom = line.split()  # 将单个数据分隔开存好
        numbers_float = list(map(float, odom))  # 转化为浮点数
        position.append([numbers_float[tx_index], numbers_float[tx_index + 1], numbers_float[tx_index + 2]])


# In[3]:


# imu真实值经过欧拉积分得到的轨迹
position1 = []
quaterntions1 = []
timestamp1 = []
with open(filepath + '/imu_true_euler.txt', 'r') as f: 

    data = f.readlines()  # txt中所有字符串读入data
    for line in data:
        odom = line.split()  # 将单个数据分隔开存好
        numbers_float = list(map(float, odom))  # 转化为浮点数
        position1.append([numbers_float[tx_index], numbers_float[tx_index + 1], numbers_float[tx_index + 2]])


# In[4]:


# imu真实值经过中值积分得到的轨迹k
position2 = []
quaterntions2 = []
timestamp2 = []
with open(filepath + '/imu_true_median.txt', 'r') as f:  

    data = f.readlines()  # txt中所有字符串读入data
    for line in data:
        odom = line.split()  # 将单个数据分隔开存好
        numbers_float = list(map(float, odom))  # 转化为浮点数
        position2.append([numbers_float[tx_index], numbers_float[tx_index + 1], numbers_float[tx_index + 2]])


# In[5]:


# 带有噪声的IMU数据的欧拉积分
position3 = []
quaterntions3 = []
timestamp3 = []
with open(filepath + '/imu_noise_euler.txt', 'r') as f:  

    data = f.readlines()  # txt中所有字符串读入data
    for line in data:
        odom = line.split()  # 将单个数据分隔开存好
        numbers_float = list(map(float, odom))  # 转化为浮点数
        position3.append([numbers_float[tx_index], numbers_float[tx_index + 1], numbers_float[tx_index + 2]])


# In[6]:


# 带有噪声的IMU数据的中值积分
position4 = []
quaterntions4 = []
timestamp4 = []
with open(filepath + '/imu_noise_median.txt', 'r') as f:  

    data = f.readlines()  # txt中所有字符串读入data
    for line in data:
        odom = line.split()  # 将单个数据分隔开存好
        numbers_float = list(map(float, odom))  # 转化为浮点数
        position4.append([numbers_float[tx_index], numbers_float[tx_index + 1], numbers_float[tx_index + 2]])


# In[7]:


# 计算误差，用向量的二范数衡量
# 积分的时候初始位置没有保留，所以从[1:4000]
e_eulur = np.array(position)[1:4000,:] - np.array(position1)
norm_E_eulur = np.linalg.norm(e_eulur,axis=1,keepdims=True)
# 中值积分的
e_median = np.array(position)[1:4000,:] - np.array(position2)
norm_E_median = np.linalg.norm(e_median,axis=1,keepdims=True)

time = np.arange(1/200, 20,1/200)
plt.plot(time,norm_E_eulur)
plt.plot(time,norm_E_median)
plt.xlabel('simulation time /s')
plt.ylabel('Error /m')
plt.legend(['Eulur','Median'])     #添加图例
plt.grid()
#  plt.show()


# In[8]:


### plot 3d

#  precision_mid = np.linalg.norm(np(position4-position))
#  precision_eulor = np.linalg.norm(np(position1-position))

fig = plt.figure()
ax = fig.gca(projection='3d')

xyz =  list(zip(*position))      #真值
xyz1 = list(zip(*position1))     #真值欧拉积分　
xyz2 = list(zip(*position2))     #真值中值积分
xyz3= list(zip(*position3))      #加入噪声的欧拉积分
xyz4= list(zip(*position4))      #加入噪声的中值积分
print
ax.plot(xyz[0], xyz[1], xyz[2], label='gt')
ax.plot(xyz1[0], xyz1[1], xyz1[2], label='raw_eulur')
ax.plot(xyz2[0], xyz2[1], xyz2[2], label='raw_median')
ax.plot(xyz3[0], xyz3[1], xyz3[2], label='noise_eulur')
ax.plot(xyz4[0], xyz4[1], xyz4[2], label='noise_median')
ax.legend()

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()


# In[ ]:




