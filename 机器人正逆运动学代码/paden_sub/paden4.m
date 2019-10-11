%该函数是Paden子问题的一个扩展
%问题描述：点p经过一系列与twist2平行的旋转轴线旋转，再经过
%twist1的旋转theta到达q，求解关于Twist1的旋转角度。
% 输出值为twist1角度的两个可能解。
%输入twist1和twist2为旋量类或者六维向量
%输出为关节角（弧度）

%参考文献：
%熊有伦等《机器人学：建模控制与视觉》

function [ theta1,theta10 ] =Paden4(twist1,twist2,p,q)
%% 输入处理
if  isa(twist1,'Twist') && isa(twist2,'Twist')
%判断输入是否符合Twist定义并获取w（轴线方向）和r（轴线上一点）
    w1=twist1.w;
    r=twist1.pole;
    w2=twist2.w;
    w1=w1(:);
    w2=w2(:);
elseif isa(twist1,'double') && isa(twist2,'double') && length(twist1)==6 ...
        && length(twist2)==6
%判断输入是否符合六维向量并获取w（轴线方向）和r（轴线上一点）
    w1=twist1(1:3)/(norm(twist1(1:3)));
    r=twist1(4:6);
    w2=twist2(1:3)/(norm(twist2(1:3)));
    w1=w1(:);
    w2=w2(:);
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
%% 问题求解
u=p-r;
v=q-r;
%求解向量z=x1*w1+x2*w2+x3cross(w1,w2)中的系数
x1=((w1'*w2)*w2'*u-w1'*v)/((w1'*w2)^2-1);
x2=((w1'*w2)*w1'*v-w2'*u)/((w1'*w2)^2-1);
x3_1=((norm(v))^2-x1^2-x2^2-2*x1*x2*w1'*w2)/(norm(cross(w1,w2))^2); 
if x3_1<-10^-8  %判断是否无解（考虑舍入误差）
    %error('起始点无法达到目标点')
    theta1=nan;
    theta10=nan;
    return
elseif x3_1>=-10^-8 && x3_1<0
    x3_1=0;
end
x3=sqrt(x3_1);
z=x1*w1+x2*w2+x3*cross(w1,w2);
c=z+r; %求交点c
theta1=Paden1(twist1,c,q);
x31=-sqrt(x3_1);
z1=x1*w1+x2*w2+x31*cross(w1,w2);
c1=z1+r; %求交点c1
theta10=Paden1(twist1,c1,q);
x=[theta1 theta10];
%保证角度在正负pi之间
for i=1:2
    if x(i)>pi
        x(i)=x(i)-2*pi;
    elseif x(i)<-pi
        x(i)=x(i)+2*pi;   
    end
end
theta1=x(1);
theta10=x(2);
end