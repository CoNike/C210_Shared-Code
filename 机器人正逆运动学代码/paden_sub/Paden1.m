function [ theta ] = Paden1( twist,p,q)
%Paden1 问题描述：一点经过旋转轴到目标点
% twist为该旋转关节的旋量，p为起始点，q为目标点；
%该问题只有一个解；
%% 问题求解
r=twist.pole;
w=twist.w;
u=p-r;
v=q-r;
u1=u-w*w'*u;
v1=v-w*w'*v;
theta=atan2(w'*cross(u1,v1),u1'*v1);
if theta<-pi
    theta=theta+2*pi;
elseif theta>pi
    theta=theta-2*pi;
end
end

