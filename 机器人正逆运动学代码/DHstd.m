function T = DHstd(alpha,a,d,theta,argument)
%此函数利用标准DH参数求相邻连杆的变换关系
%argument用于确定alpha和theta用弧度制还是角度制
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
    error("角度声明有误")
end
T=T1*T2;
T=T.T;
end

