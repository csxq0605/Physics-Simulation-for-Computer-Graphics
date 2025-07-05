# Lab 4 报告

## Position Based Fluids Implementation

![Free Fall](../gif/lab4_1.gif)

![Wave](../gif/lab4_2.gif)

### Introduction

Lab 参考 [Position Based Fluids](https://dl.acm.org/doi/10.1145/2461912.2461984) 论文，实现了 3D 的 PBF 仿真。

### Implementation Details

Lab 实现了论文中的：

- Enforcing incompressibility
- Artificial pressure
- XSPH viscosity

由于计算资源等方面的限制，实现中的 Settings （如粒子数，时间步长等）难以与论文中保持一致，而 PBF 方法包含诸多无明确物理意义的参数（10个左右），因此这些参数都需要细致的改变。为在 Lab 框架上尽可能实现更好表现的实时模拟，我在对参数的调节上付出了一定努力。

具体代码及参数见 [FluidSimulator.h](../src/VCX/Labs/4-PBF/FluidSimulator.h) 和 [FluidSimulator.cpp](../src/VCX/Labs/4-PBF/FluidSimulator.cpp) 。

### Optimization

Lab 中首先使用了 Uniform Grid Hash 加速 Neighbor Search 过程，此时 FPS 在 20 左右。进一步使用多线程对计算量较大的部分加速后， FPS 提升到 35 左右，加速了超过 1.5 倍。多线程实现的代码见 [parallel.h](../src/VCX/Labs/4-PBF/parallel.h) ，其中经过实验将线程数手动设置为 8 。实验均在笔记本上进行。

### Scene and Interaction

Lab 展示了两个场景的表现，分别为液体自由下落和人工产生的波浪。流体粒子被渲染为不同深浅的蓝色，粒子的邻居越多，颜色越深。球形障碍物可以通过按住 Alt 键使用鼠标移动。在左下角可以调节液体大小和相对位置，以及挡板的移速。
