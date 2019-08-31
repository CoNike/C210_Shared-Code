function [ theta ] = Paden1( twist,p,q)
%Paden1 ����������һ�㾭����ת�ᵽĿ���
% twistΪ����ת�ؽڵ�������pΪ��ʼ�㣬qΪĿ��㣻
%������ֻ��һ���⣻
%% �������
r=twist.pole;
w=twist.w;
u=p-r;
v=q-r;
u1=u-w*w'*u;
v1=v-w*w'*v;
theta=atan2(w'*cross(u1,v1),u1'*v1);
if theta<-pi
    theta=theta+2*pi;
elseif theta>pi
    theta=theta-2*pi;
end
end

