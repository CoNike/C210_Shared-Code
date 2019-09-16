### 正逆运动学说明
***
---
目前有的机器逆解人：
> UR5 (Matlab / C++)
  
> MH5F安川机器人  (Matlab / C++)

1.Matlab文件可以直接运行出结果。

2.C++ 文档采用的是RobWork框架，需要使用RobWork C++库进行编译。

3.每次上传新的逆解之后，请修改ReadMe.md文档。

4.paden_sub是指数积求解逆解是需要用到的Paden子问题，后续还需要添加.

5.some_robot文件中包括一些机器人正逆运动学算法，包括ABB660，AUBO，安川MH5F和数值解的测试算法

6.DH相关的文件为利用DH建模求解相邻连杆的变换矩阵，fkine_screw.m为基于旋量的正运动学求解。

7.ikine_num.m为机器人的数值迭代解求解的测试算法，存在一定的冗余性

8.jaco.m，jacobin0.m,jacobi_screw.m为雅克比矩阵求解的相关算法

9.POE文件夹中为指数积建模的算法和测试文档
