%Paden2 问题描述：一点绕两个有序（顺序为twist2,twist1)的相交轴到指定点
%twist1、twist2为两个旋量，p、q为起始点和目标点，
%r为两旋量的tw1st1、twist2交点。
%theta1和theta10为旋转轴1可能的两个角度
%theta2和theta20为旋转轴2可能的两个角度
%输入twist1、twist2为旋量类或者六维向量
%输出为theta为关节角（弧度）

%参考文献：
%熊有伦等《机器人学：建模控制与视觉》

function [ theta1,theta10,theta2,theta20] = Paden2( twist1,twist2,p,q,r)
%% 输入处理
if nargin == 4                  
%判断输入参数是否包含交点r，若无，则生成，同时判断输入是否正确
    r0=twistcross(twist1,twist2); %调用twistcross函数，生成SE3下交点r0
    r=r0(1:3);                    %转换为笛卡尔空间坐标系下的三维向量点
end
if  isa(twist1,'Twist') && isa(twist2,'Twist')
%判断输入是否符合Twist定义并获取w（轴线方向）和r（轴线上一点）
    w1=twist1.w;
    w2=twist2.w;
    w1=w1(:);
    w2=w2(:);
elseif isa(twist1,'double') && isa(twist2,'double') && length(twist1)==6 ...
        && length(twist2)==6
%判断输入是否符合六维向量并获取w（轴线方向）和r（轴线上一点）
    w1=twist1(1:3)/(norm(twist1(1:3)));
    w2=twist2(1:3)/(norm(twist2(1:3)));
    w1=w1(:);
    w2=w2(:);
else
    error('输入格式不对')
end
%判断输入点是否符合要求
if isa(p,'double') && isa(q,'double') && isa(r,'double') && length(p)==3 ... 
        && length(q)==3 && length(r)==3
    p=p(:);
    q=q(:);
    r=r(:);
else
    error('输入格式不对')
end
%% 求解过程
u=p-r;
v=q-r;
%求解向量z=x1*w1+x2*w2+x3cross(w1,w2)中的系数
x1=((w1'*w2)*w2'*u-w1'*v)/((w1'*w2)^2-1);
x2=((w1'*w2)*w1'*v-w2'*u)/((w1'*w2)^2-1);
x3_1=((norm(u))^2-x1^2-x2^2-2*x1*x2*w1'*w2)/(norm(cross(w1,w2))^2); 
if x3_1<-10^-8  %判断是否无解（考虑舍入误差）
    theta1=nan;
    theta10=nan;
    theta2=nan;
    theta20=nan;
    return
elseif x3_1>=-10^-8 && x3_1<0
    x3_1=0;
end
x3=sqrt(x3_1);
z=x1*w1+x2*w2+x3*cross(w1,w2);
c=z+r; %求交点c
theta2=Paden1(twist2,p,c);
theta1=Paden1(twist1,c,q);
x31=-sqrt(x3_1);
z1=x1*w1+x2*w2+x31*cross(w1,w2);
c1=z1+r; %求交点c1
theta20=Paden1(twist2,p,c1);
theta10=Paden1(twist1,c1,q);
x=[theta1 theta10 theta2 theta20];
%保证角度在正负pi之间
for i=1:4
    if x(i)>pi
        x(i)=x(i)-2*pi;
    elseif x(i)<-pi
        x(i)=x(i)+2*pi;   
    end
end
theta1=x(1);
theta10=x(2);
theta2=x(3);
theta20=x(4);
 end