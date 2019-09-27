%Paden2 ����������һ������������˳��Ϊtwist2,twist1)���ཻ�ᵽָ����
%twist1��twist2Ϊ����������p��qΪ��ʼ���Ŀ��㣬
%rΪ��������tw1st1��twist2���㡣
%theta1��theta10Ϊ��ת��1���ܵ������Ƕ�
%theta2��theta20Ϊ��ת��2���ܵ������Ƕ�
%����twist1��twist2Ϊ�����������ά����
%���ΪthetaΪ�ؽڽǣ����ȣ�

%�ο����ף�
%�����׵ȡ�������ѧ����ģ�������Ӿ���

function [ theta1,theta10,theta2,theta20] = Paden2( twist1,twist2,p,q,r)
%% ���봦��
if nargin == 4                  
%�ж���������Ƿ��������r�����ޣ������ɣ�ͬʱ�ж������Ƿ���ȷ
    r0=twistcross(twist1,twist2); %����twistcross����������SE3�½���r0
    r=r0(1:3);                    %ת��Ϊ�ѿ����ռ�����ϵ�µ���ά������
end
if  isa(twist1,'Twist') && isa(twist2,'Twist')
%�ж������Ƿ����Twist���岢��ȡw�����߷��򣩺�r��������һ�㣩
    w1=twist1.w;
    w2=twist2.w;
    w1=w1(:);
    w2=w2(:);
elseif isa(twist1,'double') && isa(twist2,'double') && length(twist1)==6 ...
        && length(twist2)==6
%�ж������Ƿ������ά��������ȡw�����߷��򣩺�r��������һ�㣩
    w1=twist1(1:3)/(norm(twist1(1:3)));
    w2=twist2(1:3)/(norm(twist2(1:3)));
    w1=w1(:);
    w2=w2(:);
else
    error('�����ʽ����')
end
%�ж�������Ƿ����Ҫ��
if isa(p,'double') && isa(q,'double') && isa(r,'double') && length(p)==3 ... 
        && length(q)==3 && length(r)==3
    p=p(:);
    q=q(:);
    r=r(:);
else
    error('�����ʽ����')
end
%% ������
u=p-r;
v=q-r;
%�������z=x1*w1+x2*w2+x3cross(w1,w2)�е�ϵ��
x1=((w1'*w2)*w2'*u-w1'*v)/((w1'*w2)^2-1);
x2=((w1'*w2)*w1'*v-w2'*u)/((w1'*w2)^2-1);
x3_1=((norm(u))^2-x1^2-x2^2-2*x1*x2*w1'*w2)/(norm(cross(w1,w2))^2); 
if x3_1<-10^-8  %�ж��Ƿ��޽⣨����������
    theta1=nan;
    theta10=nan;
    theta2=nan;
    theta20=nan;
    return
elseif x3_1>=-10^-8 && x3_1<0
    x3_1=0;
end
x3=sqrt(x3_1);
z=x1*w1+x2*w2+x3*cross(w1,w2);
c=z+r; %�󽻵�c
theta2=Paden1(twist2,p,c);
theta1=Paden1(twist1,c,q);
x31=-sqrt(x3_1);
z1=x1*w1+x2*w2+x31*cross(w1,w2);
c1=z1+r; %�󽻵�c1
theta20=Paden1(twist2,p,c1);
theta10=Paden1(twist1,c1,q);
x=[theta1 theta10 theta2 theta20];
%��֤�Ƕ�������pi֮��
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