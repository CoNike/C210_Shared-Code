%Padene 问题描述：点p经过有序的相交旋转轴（twist2,twist1为相关轴线的运动旋量）
%到离q1距离de1和q2距离为de2的点
%r0为两个旋量的交点
%输出为四组可能的关节角度
%输入twist1和twist2为旋量类或者六维向量
%输出为关节角（弧度）

%参考文献：
%熊有伦等《机器人学：建模控制与视觉》

function [ q01,q02,q03,q04 ] = Padene(twist1,twist2,p,q1,q2,r0,de1,de2)
%% 输入处理
if nargin == 7 
%判断输入参数是否包含交点r0，若无，则生成，同时判断输入是否正确
    r=twistcross(twist1,twist2); %调用twistcross函数，生成SE3下交点r0
    r0=r(1:3);                   %转换为笛卡尔空间坐标系下的三维向量点
end
%判断输入是否符合Twist定义或者是否为六维向量
if  isa(twist1,'Twist') && isa(twist2,'Twist')
elseif isa(twist1,'double') && isa(twist2,'double') && length(twist1)==6 ...
        && length(twist2)==6
else
    error('输入格式不对')
end
if isa(p,'double') && isa(q1,'double') && isa(q2,'double') && isa(r0,'double') ... 
        && length(p)==3 && length(q1)==3 && length(q2)==3 && length(r0)==3
%判断输入点是否符合要求
    p=p(:);
    q1=q1(:);
    q2=q2(:);
    r0=r0(:);
else
    error('输入格式不对')
end
%% 求解过程
%求解三个球的交点
w=p-r0;
u=q1-r0;
v=q2-r0;
de3=norm(w);
de4=norm(u);
de5=norm(v);
%单位化处理
u1=u/(norm(u));
v1=v/(norm(v));

t1=de3*((de3^2+de4^2-de1^2)/(2*de3*de4));
t2=de3*((de3^2+de5^2-de2^2)/(2*de3*de5));
%求解z=x1*u1+x2*v1+x3*cross(u1,v1)中的系数
x1=(t1-u1'*v1*t2)/(1-(u1'*v1)^2);
x2=(t2-u1'*v1*t1)/(1-(u1'*v1)^2);
x3_1=(de3^2-x1^2-x2^2-2*x1*x2*u1'*v1)/(norm(cross(u1,v1))^2);
if abs(x3_1-0)<0.01*min(abs(x1),abs(x2)) %判断是否无解（考虑舍入误差）
    x3=0;
elseif x3_1>0
    x3=sqrt(x3_1);
else
    %     disp('there is no answer');
    q01=nan;
    q02=nan;
    q03=nan;
    q04=nan;
    return
end     
z=x1*u1+x2*v1+x3*cross(u1,v1);
q=z+r0;
[theta10,theta11,theta20,theta21]=Paden2(twist1,twist2,p,q,r0);
z1=x1*u1+x2*v1-x3*cross(u1,v1);
q1=z1+r0;
[theta12,theta13,theta22,theta23]=Paden2(twist1,twist2,p,q1,r0);
q01=[theta10,theta20];
q02=[theta11,theta21];   
q03=[theta12,theta22];  
q04=[theta13,theta23];   
end
