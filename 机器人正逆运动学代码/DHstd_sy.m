function T = DHstd_sy(alpha,a,d,theta)
%此函数利用标准DH参数求相邻连杆的变换关系
%可以用于符号计算，但是在符号计算时注意pi的问题
%输入的alpha和theta单位为rad
T = [cos(theta),   -sin(theta)*cos(alpha),  sin(alpha)*sin(theta),  a*cos(theta);
    sin(theta),     cos(theta)*cos(alpha), -sin(alpha)*cos(theta),  a*sin(theta);
    0,                         sin(alpha),             cos(alpha),             d;
    0,                                  0,                      0,            1];
end