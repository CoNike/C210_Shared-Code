%Paden1 问题描述：一点经过旋转轴到目标点
% twist为该旋转关节的旋量，p为起始点，q为目标点；
%该问题只有一个解；
%输入twist为旋量类或者六维向量；
%输出为theta为关节角（弧度）；

%参考文献：
%熊有伦等《机器人学：建模控制与视觉》

function [ theta ] = Paden1( twist,p,q)
%% 输入处理
if  isa(twist,'Twist') 
%判断输入是否符合Twist定义并获取w（轴线方向）和r（轴线上一点）
    r=twist.pole;
    w=twist.w;
    w=w(:);
    r=r(:);
elseif isa(twist,'double') && length(twist)==6
%判断输入是否符合六维向量并获取w（轴线方向）和r（轴线上一点）
    w=twist(1:3)/(norm(twist(1:3)));
    r=twist(4:6);
    w=w(:);
    r=r(:);
else
    error('输入格式不对')
end
%判断输入点是否符合要求
if isa(p,'double') && isa(q,'double') && length(p)==3 ...
        && length(q)==3
    p=p(:);
    q=q(:);
else
    error('输入格式不对')
end
%% 问题求解
u=p-r;
v=q-r;
%求u和v在垂直于w方向的投影
u1=u-w*w'*u;
v1=v-w*w'*v;
if abs(w'*u-w'*v) > 10^-5 || abs(norm(u1)-norm(v1))>10^-5 %判断是否无解
    theta=nan;
    return
end   
theta=atan2(w'*cross(u1,v1),u1'*v1);
%确保theta在正负pi之间
if theta<-pi
    theta=theta+2*pi;
elseif theta>pi
    theta=theta-2*pi;
end
end