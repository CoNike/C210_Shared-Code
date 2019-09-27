%�ú�����Paden�������һ����չ
%������������p����һϵ����twist2ƽ�е���ת������ת���پ���
%twist1����תtheta����q��������Twist1����ת�Ƕȡ�
% ���ֵΪtwist1�Ƕȵ��������ܽ⡣
%����twist1��twist2Ϊ�����������ά����
%���Ϊ�ؽڽǣ����ȣ�

%�ο����ף�
%�����׵ȡ�������ѧ����ģ�������Ӿ���

function [ theta1,theta10 ] =Paden4(twist1,twist2,p,q)
%% ���봦��
if  isa(twist1,'Twist') && isa(twist2,'Twist')
%�ж������Ƿ����Twist���岢��ȡw�����߷��򣩺�r��������һ�㣩
    w1=twist1.w;
    r=twist1.pole;
    w2=twist2.w;
    w1=w1(:);
    w2=w2(:);
elseif isa(twist1,'double') && isa(twist2,'double') && length(twist1)==6 ...
        && length(twist2)==6
%�ж������Ƿ������ά��������ȡw�����߷��򣩺�r��������һ�㣩
    w1=twist1(1:3)/(norm(twist1(1:3)));
    r=twist1(4:6);
    w2=twist2(1:3)/(norm(twist2(1:3)));
    w1=w1(:);
    w2=w2(:);
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
%% �������
u=p-r;
v=q-r;
%�������z=x1*w1+x2*w2+x3cross(w1,w2)�е�ϵ��
x1=((w1'*w2)*w2'*u-w1'*v)/((w1'*w2)^2-1);
x2=((w1'*w2)*w1'*v-w2'*u)/((w1'*w2)^2-1);
x3_1=((norm(v))^2-x1^2-x2^2-2*x1*x2*w1'*w2)/(norm(cross(w1,w2))^2); 
if x3_1<-10^-8  %�ж��Ƿ��޽⣨����������
    %error('��ʼ���޷��ﵽĿ���')
    theta1=nan;
    theta10=nan;
    return
elseif x3_1>=-10^-8 && x3_1<0
    x3_1=0;
end
x3=sqrt(x3_1);
z=x1*w1+x2*w2+x3*cross(w1,w2);
c=z+r; %�󽻵�c
theta1=Paden1(twist1,c,q);
x31=-sqrt(x3_1);
z1=x1*w1+x2*w2+x31*cross(w1,w2);
c1=z1+r; %�󽻵�c1
theta10=Paden1(twist1,c1,q);
x=[theta1 theta10];
%��֤�Ƕ�������pi֮��
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