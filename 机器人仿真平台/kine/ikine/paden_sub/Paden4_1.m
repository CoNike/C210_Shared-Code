function [ theta11,theta12] = Paden4( twist1,twist2,p,q,r)
%Paden4����������һ��p����������ת(������ת������ƽ��)������һ��c���پ���һ�η�ƽ�����ߵ���ת������֪��q
%twist1��twist2Ϊ����������p��qΪ��ʼ������յ㣬qΪ�м�����
%theta11��theta12Ϊ��ת��1���ܵ������Ƕ�
%% ������
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
