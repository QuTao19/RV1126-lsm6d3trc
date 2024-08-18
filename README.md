# LSM6DS3TR-C陀螺仪
本项目为在RV1126平台上实现lsm6ds3tr-c芯片的姿态解算,包含驱动程序和应用程序。
解算采用了互补滤波（Mahony）算法,采用四元数的方式表示,通过PI参数来调节姿态解算过程中，
加速度计和陀螺仪的权重。

## 编译
#### 驱动编译(注意修改kernel路径)
    
    make ARCH=arm
#### 应用程序编译(注意选择交叉编译工具链路径)

    /opt/atk-dlrv1126-toolchain/bin/arm-linux-gnueabihf-gcc lsm6ds3trcApp.c -o lsm6ds3trcApp


## 参考

[STMicroelectronics/lsm6ds3-pid](https://github.com/STMicroelectronics/lsm6ds3-pid)

[mahony姿态解算算法](https://www.cnblogs.com/WangHongxi/p/12357230.html)