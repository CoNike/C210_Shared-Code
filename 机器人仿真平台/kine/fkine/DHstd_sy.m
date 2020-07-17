function T = DHstd_sy(alpha,a,d,theta)
%�˺������ñ�׼DH�������������˵ı任��ϵ
%�����alpha��theta��λΪrad
T = [cos(theta),   -sin(theta)*cos(alpha),  sin(alpha)*sin(theta),  a*cos(theta);
    sin(theta),     cos(theta)*cos(alpha), -sin(alpha)*cos(theta),  a*sin(theta);
    0,                         sin(alpha),             cos(alpha),             d;
    0,                                  0,                      0,            1];
end