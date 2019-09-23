 function [ theta1,theta10,theta2,theta20] = Paden2( twist1,twist2,p,q,r)
%Paden2 问题描述：一点绕两个有序（顺序为twist2,twist1)的相交轴到指定点
%twist1、twist2为两个旋量，p、q为起始点和目标点，
%r为两旋量的tw1st1、twist2交点。
%theta1和theta10为旋转轴1可能的两个角度
%theta2和theta20为旋转轴2可能的两个角度
%% 求解过程
%w1,w2为对应旋转轴上的点
w1=twist1.w;
w2=twist2.w;
u=p-r;
v=q-r;
x1=((w1'*w2)*w2'*u-w1'*v)/((w1'*w2)^2-1);
x2=((w1'*w2)*w1'*v-w2'*u)/((w1'*w2)^2-1);
x3_1=((norm(u))^2-x1^2-x2^2-2*x1*x2*w1'*w2)/(norm(cross(w1,w2))^2); 
x3=sqrt(x3_1);
z=x1*w1+x2*w2+x3*cross(w1,w2);
c=z+r;
theta2=Paden1(twist2,p,c);
theta1=Paden1(twist1,c,q);
x31=-sqrt(x3_1);
z1=x1*w1+x2*w2+x31*cross(w1,w2);
c1=z1+r;
theta20=Paden1(twist2,p,c1);
theta10=Paden1(twist1,c1,q);
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

