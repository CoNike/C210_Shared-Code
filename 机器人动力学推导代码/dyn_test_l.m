% 拉格朗日动力学方程求解测试
% clc;
% clear;
syms q1 q2 q3 m1 m2 m3 d2 real
syms Ix1 Iy1 Iz1 Ixy1 Iyz1 Ixz1 real
syms Ix2 Iy2 Iz2 Ixy2 Iyz2 Ixz2 real
syms Ix3 Iy3 Iz3 Ixy3 Iyz3 Ixz3 real
syms xc1 yc1 zc1 xc2 yc2 zc2 xc3 yc3 zc3 real

dh_params = [-pi/2, 0,  0, q1; 
             pi/2, 0, d2, q2;
             0, 0, 0, q3];
mass_center = [xc1, yc1, zc1; 
               xc2, yc2, zc2;
               xc3, yc3, zc3];
mass = [m1,m2,m3];
inertia_1 = [Ix1,  Ixy1, Ixz1;
             Ixy1, Iy1,  Iyz1;
             Ixz1, Iyz1, Iz1];
inertia_2 = [Ix2,  Ixy2, Ixz2;
             Ixy2, Iy2,  Iyz2;
             Ixz2, Iyz2, Iz2];
inertia_3 = [Ix3,  Ixy3, Ixz3;
             Ixy3, Iy3,  Iyz3;
             Ixz3, Iyz3, Iz3];
         
inertia_tensor(:,:,1) = inertia_1;
inertia_tensor(:,:,2) = inertia_2;
inertia_tensor(:,:,3) = inertia_3;

[h,c,g] = LagrangianDynamics(dh_params, mass, mass_center, inertia_tensor)
