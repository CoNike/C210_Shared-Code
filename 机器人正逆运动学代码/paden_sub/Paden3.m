%Paden3 问题描述：一点经过旋转轴旋转到离点q距离为De的位置
%twist为旋转关节所代表的旋量，p为起点
%theta1和theta为可能的角度
%输入twist为旋量类或者六维向量
%输出为关节角（弧度）

%参考文献：
%熊有伦等《机器人学：建模控制与视觉》

function [ theta,theta1 ] = Paden3( twist,p,q,De)
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
if isa(p,'double') && isa(q,'double') && length(p)==3 ...
        && length(q)==3
%判断输入点是否符合要求
    p=p(:);
    q=q(:);
else
    error('输入格式不对')
end
%% 求解过程
u=p-r;
v=q-r;
%%求u和v在垂直于w方向的投影
u1=u-w*w'*u;
v1=v-w*w'*v;
De_0=De^2-(w'*(p-q))^2;
De1=sqrt(De_0);
if norm(u1)+norm(v1) < De1 || abs(norm(u1)-norm(v1)) > De1 %判断是否无解
    theta=nan;
    theta1=nan;
    return
end
theta_0=atan2(w'*cross(u1,v1),u1'*v1);
t=acos((norm(u1)^2+norm(v1)^2-De1^2)/(2*norm(u1)*norm(v1)));
theta1=theta_0+t;
theta=theta_0-t;
%保证角度在正负pi之间
if theta1 <= -pi
    theta1=theta1+2*pi;
elseif theta1>pi
    theta1=theta1-2*pi;
end
if theta<=-pi
    theta=theta+2*pi;
elseif theta>pi
    theta=theta-2*pi;
end
