%Paden1 ����������һ�㾭����ת�ᵽĿ���
% twistΪ����ת�ؽڵ�������pΪ��ʼ�㣬qΪĿ��㣻
%������ֻ��һ���⣻
%����twistΪ�����������ά������
%���ΪthetaΪ�ؽڽǣ����ȣ���

%�ο����ף�
%�����׵ȡ�������ѧ����ģ�������Ӿ���

function [ theta ] = Paden1( twist,p,q)
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
%�ж�������Ƿ����Ҫ��
if isa(p,'double') && isa(q,'double') && length(p)==3 ...
        && length(q)==3
    p=p(:);
    q=q(:);
else
    error('�����ʽ����')
end
%% �������
u=p-r;
v=q-r;
%��u��v�ڴ�ֱ��w�����ͶӰ
u1=u-w*w'*u;
v1=v-w*w'*v;
if abs(w'*u-w'*v) > 10^-5 || abs(norm(u1)-norm(v1))>10^-5 %�ж��Ƿ��޽�
    theta=nan;
    return
end   
theta=atan2(w'*cross(u1,v1),u1'*v1);
%ȷ��theta������pi֮��
if theta<-pi
    theta=theta+2*pi;
elseif theta>pi
    theta=theta-2*pi;
end
end