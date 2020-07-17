function [ theta1,theta10,theta2,theta20] = Paden5( twist1,twist2,p,q)
%问题描述：一点绕两个平行轴连续旋转两次到达指定点
%twist1、twist2为两个旋量，p、q为起始点和目标点
%r1和r2分别为两旋量twist1、twist2的圆心点
%theta1和theta10为旋转轴1可能的两个角度
%theta2和theta20为旋转轴2可能的两个角度
%% 求解过程
w1=twist1.w;
w2=twist2.w;
m1=twist1.pole;
m2=twist2.pole;
u1=p-m1;
v1=q-m2;
r1=w1'*u1+m1;
r2=w2'*v1+m2;
r12=r1-r2;
r21=r2-r1;
u=p-r1;
v=q-r2;
theta01=atan2(w1'*cross(u,r21),u'*r21);
n1=((norm(u))^2+(norm(r21))^2-(norm(v))^2)/(2*norm(u)*norm(r21));
if n1>=1
    n1=1;
elseif n1<=-1
    n1=-1;
end
fai1=acos(n1);
theta1=theta01+fai1;
theta10=theta01-fai1;
theta02=atan2(w2'*cross(v,r12),v'*r12);
m1=((norm(v))^2+(norm(r12))^2-(norm(u))^2)/(2*norm(v)*norm(r12));
if m1>=1
    m1=1;
elseif m1<=-1
    m1=-1;
end
fai2=acos(m1);
theta2=theta02+fai2;
theta20=theta02-fai2;
% 避免theta1角度超出范围
if theta1<=-pi
    theta1=theta1+2*pi;
end
if theta10<=-pi
    theta10=theta10+2*pi;
end
if theta1>=pi
    theta1=theta1-2*pi;
end
if theta10>=pi
    theta10=theta10-2*pi;
end
% 避免theta2角度超出范围
if theta2<=-pi
    theta2=theta2+2*pi;
end
if theta20<=-pi
    theta20=theta20+2*pi;
end
if theta2>=pi
    theta2=theta2-2*pi;
end
if theta20>=pi
    theta20=theta20-2*pi;
end
end