function [ theta,theta1 ] = Paden3( twist,p,q,De)
%Paden3 问题描述：一点经过旋转轴旋转到离点q距离为De的位置
%twist为旋转关节所代表的旋量，p为起点
%theta1和theta为可能的角度
%% 求解过程
r=twist.pole;
w=twist.w;
u=p-r;
v=q-r;
u1=u-w*w'*u;
v1=v-w*w'*v;
De_0=De^2-(w'*(p-q))^2;
De1=sqrt(De_0);
theta_0=atan2(w'*cross(u1,v1),u1'*v1);
t=acos((norm(u1)^2+norm(v1)^2-De1^2)/(2*norm(u1)*norm(v1)));
theta1=theta_0+t;
theta=theta_0-t;
% 避免出现角度太小，
if theta1<=-pi
    theta1=theta+2*pi;
end
if theta<=-pi
    theta=theta+2*pi;
end
if theta1>=pi
    theta1=theta-2*pi;
end
if theta>=pi
    theta=theta-2*pi;
end
