function [ theta,theta1 ] = Paden3( twist,p,q,De)
%Paden3 ����������һ�㾭����ת����ת�����q����ΪDe��λ��
%twistΪ��ת�ؽ��������������pΪ���
%theta1��thetaΪ���ܵĽǶ�
%% ������
r=twist.pole;
w=twist.w;
u=p-r;
v=q-r;
u1=u-w*w'*u;
v1=v-w*w'*v;
De_0=De^2-(w'*(p-q))^2;
De1=sqrt(De_0);
theta_0=atan2(w'*cross(u1,v1),u1'*v1);
t=acos((norm(u1)^2+norm(v1)^2-De1^2)/(2*norm(u1)*norm(v1)));
theta1=theta_0+t;
theta=theta_0-t;
% ������ֽǶ�̫С��
if theta1<=-pi
    theta1=theta+2*pi;
end
if theta<=-pi
    theta=theta+2*pi;
end
if theta1>=pi
    theta1=theta-2*pi;
end
if theta>=pi
    theta=theta-2*pi;
end
