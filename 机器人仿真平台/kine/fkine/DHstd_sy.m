function T = DHstd_sy(alpha,a,d,theta)
%此函数利用标准DH参数求相邻连杆的变换关系
%输入的alpha和theta单位为rad
T = [cos(theta),   -sin(theta)*cos(alpha),  sin(alpha)*sin(theta),  a*cos(theta);
    sin(theta),     cos(theta)*cos(alpha), -sin(alpha)*cos(theta),  a*sin(theta);
    0,                         sin(alpha),             cos(alpha),             d;
    0,                                  0,                      0,            1];
end