function [ theta11,theta12] = Paden4( twist1,twist2,p,q,r)
%Paden4问题描述：一点p经过三次旋转(三次旋转的轴线平行)到达另一点c后再经过一次非平行轴线的旋转到达已知点q
%twist1、twist2为两个旋量，p、q为起始点和最终点，q为中间待求点
%theta11和theta12为旋转轴1可能的两个角度
%% 求解过程
w1=twist1.w;
w2=twist2.w;
u=p-r;
v=q-r;
x1=((w1'*w2)*w2'*u-w1'*v)/((w1'*w2)^2-1);
x2=((w1'*w2)*w1'*v-w2'*u)/((w1'*w2)^2-1);
x3=((norm(v))^2-x1^2-x2^2-2*x1*x2*w1'*w2)/(norm(cross(w1,w2))^2); 
x31=sqrt(x3);
z1=x1*w1+x2*w2+x31*cross(w1,w2);
c1=z1+r;
theta11=Paden1(twist1,c1,q);
x32=-sqrt(x3);
z2=x1*w1+x2*w2+x32*cross(w1,w2);
c2=z2+r;
theta12=Paden1(twist1,c2,q);
end
