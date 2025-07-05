# Lab 3 报告

## 软体仿真

![case1](../gif/lab3_1.gif)

Lab 完成了软体仿真的所有要求。在左下角可以选择能量模型和调整物理参数。可以按住 Alt 键使用鼠标左键对软体施加外力，其中力的作用点为查找与鼠标对应的射线距离在一定范围内且最近的点。具体代码见[ExternalForceManager.cpp](../src/VCX/Labs/Common/ExternalForceManager.cpp)。

Lab 实现了 StVK 和 Neo-Hookean 两个能量模型。可以看到它们受到较大外力压缩时展现出的不同特性。

## 布料仿真

![case2](../gif/lab3_2.gif)

Lab 完成了布料仿真的所有要求。交互与软体仿真相同。由于没有实现光照，可以开启`Show Grid`以更清楚地观察布料的状态。
