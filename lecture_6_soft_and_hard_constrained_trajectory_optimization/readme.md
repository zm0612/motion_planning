# 实验报告

由于本节的内容理论并不复杂，主要重在理解。因此我并有花费太多时间在代码的实现上，我只是完成了matlab的代码部分，目的就是理解基于凸优化的贝塞尔曲线生成。

基于贝塞尔的曲线生成实验结果如下图：

<img src="doc/bezier.jpg" alt="bezier" style="zoom:50%;" />

> 如果需要对曲线生成的代码进行C++实现，需要使用关于QP求解库，[osqp](https://github.com/osqp/osqp)也许是一个不错的选择。