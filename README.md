# KF_GINS_Learning

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

### 3、kf_gins main.cpp
读取yaml配置文件，得到imu和GNSS定位结果路径

首先判断命令行参数，如果不为 2（可执行程序名算第一个参数 `argv[0]`）即没传入配置文件路径，输出提示并退出程序：
1.命令行参数为可执行程序的路径 2.第二个命令行参数为可执行程序的yaml配置文件

```cpp
if (argc != 2) {
    std::cout << "usage: KF-GINS kf-gins.yaml" << std::endl;
    return -1;
}
```

基于absl:Now函数获得当前程序运行的电脑时间，基于北京时区的时间
```cpp
auto ts = absl::Now();
```

YAML库尝试读取配置文件，如何读取不成功LoadFile会给出Badfile exception信息，会输出读取配置文件失败。如果读取成功，返回一个分配内存空间的config对象。
```cpp
    YAML::Node config;
    try {
        config = YAML::LoadFile(argv[1]);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to read configuration file. Please check the path and format of the configuration file!"
                  << std::endl;
        return -1;
    }
```

配置选项类
![image-20230925183309602](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925183309602.png)

创建一个GINSOptions结构体对象，里面存储配置文件中的误差参数、初始位置和初始姿态，IMU误差模型参数（随机游走和零篇不稳定性）
```cpp
    GINSOptions options;
    if (!loadConfig(config, options)) {
        std::cout << "Error occurs in the configuration file!" << std::endl;
        return -1;
    }
```
