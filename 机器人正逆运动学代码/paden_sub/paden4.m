function [ theta1,theta10 ] =paden4(twist1,twist2,p,q)
%�ú�����Paden�������һ����չ
%������������p����һϵ����Twist2ƽ�е���ת������ת���پ���
%Twist1����תtheta����q��������Twist1����ת�Ƕȡ�
% ���ֵΪ����1�Ƕȵ��������ܽ⡣
w1=twist1.w;
r=twist1.pole;
w2=twist2.w;
u=p-r;
v=q-r;
x1=((w1'*w2)*w2'*u-w1'*v)/((w1'*w2)^2-1);
x2=((w1'*w2)*w1'*v-w2'*u)/((w1'*w2)^2-1);
x3_1=((norm(v))^2-x1^2-x2^2-2*x1*x2*w1'*w2)/(norm(cross(w1,w2))^2); 

if x3_1<0
    error('��ʼ���޷��ﵽĿ���')
end
x3=sqrt(x3_1);
z=x1*w1+x2*w2+x3*cross(w1,w2);
c=z+r;
theta1=Paden1(twist1,c,q);
x31=-sqrt(x3_1);
z1=x1*w1+x2*w2+x31*cross(w1,w2);
c1=z1+r;
theta10=Paden1(twist1,c1,q);
x=[theta1 theta10];
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

