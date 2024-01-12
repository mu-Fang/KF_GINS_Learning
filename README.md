# KF_GINS_Learning

## kf_gins mian cpp
读取yaml配置文件，得到imu和GNSS定位结果路径

### 1、函数调用关系
![image-20230925181044694](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925181044694.png)

![image-20230925181228939](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925181228939.png)

### 2、重点函数

* `newImuProcess()` 是松组合的核心函数。
* `isToUpdate()` 中根据当前 IMU 和 GNSS 时间戳关系，判断要不要进行 GNSS 量测更新。
* `imuCompensate()` 中进行 IMU 校正，即减去零偏、除以比例。
* `insPropagation()` 中实现捷联惯导 PVA 和噪声递推、构建 F 矩阵。
* `insMech()` 中 IMU 机械编排，依次进行速度更新、位置更新、姿态更新。
* `gnssUpdate()` 中进行 GNSS 量测更新，实现杆臂补偿。
* `stateFeedback()` GNSS 量测更新后，状态向量误差反馈。
