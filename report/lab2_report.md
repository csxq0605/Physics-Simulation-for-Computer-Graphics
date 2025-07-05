# Lab 2 报告

![lab2_1](../gif/lab2_1.gif)

![lab2_2](../gif/lab2_2.gif)

此 Lab 实现中完成了所有的要求和可选功能。在左下角可以调整时间步长和Flip Ratio。由于Lab框架基本上是实时模拟，且模拟时间远大于渲染时间，因此难以既不降低帧率又缩短模拟的时间步长，比如若增加`numSubSteps`，事实上FPS会明显下降而每步模拟的时间步长基本没有变化。因此这里采用了类似慢放的想法，通过调整`Slow Motion`“慢放”倍率，将模拟的时间步长变为`Engine::GetDeltaTime() / _simulation.slowMotion`（具体代码见[CaseFluid.cpp](../src/VCX/Labs/2-FluidSimulation/CaseFluid.cpp)中`CaseFluid::OnRender`的实现），这样FPS无变化的同时可以缩短时间步长（虽然并没有看出模拟效果明显的提升）。

流体粒子被渲染为不同深浅的蓝色，单元格的粒子密度越高，粒子的颜色越深，代码见[FluidSimulator.cpp](../src/VCX/Labs/2-FluidSimulation/FluidSimulator.cpp)中`Simulator::updateParticleColors`的实现。红色球形障碍物可以通过按住 Alt 键使用鼠标移动。
