function T=DHmod(alpha,a,d,theta,argument)
%基于改进DH方法求解相邻连杆之间的变换矩阵
%argument是用于确定alpha和theta采用的是弧度制还是角度制
if nargin==4
    argument='deg';
end
if strcmp(argument,'deg')
    T1=SE3.Rz(theta,'deg')*SE3(0,0,d);
    T2=SE3.Rx(alpha,'deg')*SE3(a,0,0);
elseif strcmp(argument,'rad')
    T1=SE3.Rz(theta)*SE3(0,0,d);
    T2=SE3.Rx(alpha)*SE3(a,0,0);
else
    error('角度声明有误')
end
T=T2*T1;
T=T.T;
end