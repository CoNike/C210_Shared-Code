%Paden2 问题描述：一点绕两个有序（顺序为twist2,twist1)的平行轴到指定点
%twist1、twist2为两个旋量，p、q为起始点和目标点，
%两个旋量平行
%theta1和theta10为旋转轴1可能的两个角度
%theta2和theta20为旋转轴2可能的两个角度
%输入twist1和twist2为旋量类或者六维向量
%输出为关节角（弧度）

%参考文献：
%熊有伦等《机器人学：建模控制与视觉》

function [ theta1,theta10,theta2,theta20] = Paden5( twist1,twist2,p,q)
%% 输入处理
if  isa(twist1,'Twist') && isa(twist2,'Twist')
%判断输入是否符合Twist定义并获取w（轴线方向）和r（轴线上一点）
    w1=twist1.w;
    r1=twist1.pole;
    w2=twist2.w;
    r2=twist2.pole;
    w1=w1(:);
    r1=r1(:);
    w2=w2(:);
    r2=r2(:);
elseif isa(twist1,'double') && isa(twist2,'double') && length(twist1)==6 ...
        && length(twist2)==6
%判断输入是否符合六维向量并获取w（轴线方向）和r（轴线上一点）
    w1=twist1(1:3)/(norm(twist1(1:3)));
    r1=twist1(4:6);
    w2=twist2(1:3)/(norm(twist2(1:3)));
    r2=twist2(4:6);    
    w1=w1(:);
    w2=w2(:);
    r1=r1(:);
    r2=r2(:);
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
u1=q-r1;
v1=p-r2;
%%求u和v在垂直于w方向的投影
u=u1-(u1'*w1)*w1;
v=v1-(v1'*w1)*w1;
%求解r1和r2在垂直于w方向的投影
r1=q-u;
r2=p-v;
de1=norm(u);
de2=norm(v);
de3=norm(r1-r2);
m=(de1^2+de3^2-de2^2)/(2*de1*de3);
if 1<=m && m<=1+10^-5 %判断是否无解（考虑舍入误差）
    m=1;
elseif -1-10^-5<=m && m<=-1
    m=-1;
end
if m>1+10^-5 || m<-1-10^-5
    theta1=nan;
    theta10=nan;
    theta2=nan;
    theta20=nan;
    return
end
alpha1=acos(m);
%求中间向量z及交点p1
zx=cos(alpha1)*de1*(r2-r1)/de3;
zy=sin(alpha1)*de1*cross((r2-r1),w1)/norm(cross((r2-r1),w1));
z=zy+zx;
p1=r1+z;
theta2=Paden1(twist2,p,p1);
theta1=Paden1(twist1,p1,q);
%求中间向量z0及交点p1=0
z0=zx-zy;
p0=r1+z0;
theta20=Paden1(twist2,p,p0);
theta10=Paden1(twist1,p0,q);
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