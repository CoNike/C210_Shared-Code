%Paden3 ����������һ�㾭����ת����ת�����q����ΪDe��λ��
%twistΪ��ת�ؽ��������������pΪ���
%theta1��thetaΪ���ܵĽǶ�
%����twistΪ�����������ά����
%���Ϊ�ؽڽǣ����ȣ�

%�ο����ף�
%�����׵ȡ�������ѧ����ģ�������Ӿ���

function [ theta,theta1 ] = Paden3( twist,p,q,De)
%% ���봦��
if  isa(twist,'Twist')
%�ж������Ƿ����Twist���岢��ȡw�����߷��򣩺�r��������һ�㣩
    r=twist.pole;
    w=twist.w;
    w=w(:);
    r=r(:);
elseif isa(twist,'double') && length(twist)==6
%�ж������Ƿ������ά��������ȡw�����߷��򣩺�r��������һ�㣩
    w=twist(1:3)/(norm(twist(1:3)));
    r=twist(4:6);
    w=w(:);
    r=r(:);
else
    error('�����ʽ����')
end
if isa(p,'double') && isa(q,'double') && length(p)==3 ...
        && length(q)==3
%�ж�������Ƿ����Ҫ��
    p=p(:);
    q=q(:);
else
    error('�����ʽ����')
end
%% ������
u=p-r;
v=q-r;
%%��u��v�ڴ�ֱ��w�����ͶӰ
u1=u-w*w'*u;
v1=v-w*w'*v;
De_0=De^2-(w'*(p-q))^2;
De1=sqrt(De_0);
if norm(u1)+norm(v1) < De1 || abs(norm(u1)-norm(v1)) > De1 %�ж��Ƿ��޽�
    theta=nan;
    theta1=nan;
    return
end
theta_0=atan2(w'*cross(u1,v1),u1'*v1);
t=acos((norm(u1)^2+norm(v1)^2-De1^2)/(2*norm(u1)*norm(v1)));
theta1=theta_0+t;
theta=theta_0-t;
%��֤�Ƕ�������pi֮��
if theta1 <= -pi
    theta1=theta1+2*pi;
elseif theta1>pi
    theta1=theta1-2*pi;
end
if theta<=-pi
    theta=theta+2*pi;
elseif theta>pi
    theta=theta-2*pi;
end
