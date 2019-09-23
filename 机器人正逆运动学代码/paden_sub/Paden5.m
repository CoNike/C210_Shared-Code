 function [ theta1,theta10,theta2,theta20] = Paden5( twist1,twist2,p,q)
%Paden2 问题描述：一点绕两个有序（顺序为twist2,twist1)的平行轴到指定点
%twist1、twist2为两个旋量，p、q为起始点和目标点，
%两个旋量平行
%theta1和theta10为旋转轴1可能的两个角度
%theta2和theta20为旋转轴2可能的两个角度
%% 求解过程
%w1,w2为对应旋转轴上的点
w1=twist1.w;
r1=twist1.pole;
w2=twist2.w;
r2=twist2.pole;
u1=q-r1;
v1=p-r2;
u=u1-(u1'*w1)*w1;
v=v1-(v1'*w1)*w1;
r1=q-u;
r2=p-v;
de1=norm(u);
de2=norm(v);
de3=norm(r1-r2);
m=(de1^2+de3^2-de2^2)/(2*de1*de3);
if m>=1
    m=1;
elseif m<=-1
    m=-1;
end
alpha1=acos(m);
% theta_1=atan2(w1'*cross(r2-r1,u),u'*(r2-r1));
% theta1=theta_1+alpha1;
% theta10=theta_1-alpha1;
% n=(de2^2+de3^2-de1^2)/(2*de2*de3);
% if n>=1
%     n=1;
% elseif n<=-1
%     n=-1;
% end
% alpha2=acos(n);
% theta_2=atan2(w2'*cross(v,r1-r2),v'*(r1-r2));
% theta2=theta_2+alpha2;
% theta20=theta_2-alpha2;
zx=cos(alpha1)*de1*(r2-r1)/de3;
zy=sin(alpha1)*de1*cross((r2-r1),w1)/norm(cross((r2-r1),w1));
z=zy+zx;
p1=r1+z;
theta2=Paden1(twist2,p,p1);
theta1=Paden1(twist1,p1,q);
z0=zx-zy;
p0=r1+z0;
theta20=Paden1(twist2,p,p0);
theta10=Paden1(twist1,p0,q);
x=[theta1 theta10 theta2 theta20];
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
