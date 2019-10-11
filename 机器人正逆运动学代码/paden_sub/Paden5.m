%Paden2 ����������һ������������˳��Ϊtwist2,twist1)��ƽ���ᵽָ����
%twist1��twist2Ϊ����������p��qΪ��ʼ���Ŀ��㣬
%��������ƽ��
%theta1��theta10Ϊ��ת��1���ܵ������Ƕ�
%theta2��theta20Ϊ��ת��2���ܵ������Ƕ�
%����twist1��twist2Ϊ�����������ά����
%���Ϊ�ؽڽǣ����ȣ�

%�ο����ף�
%�����׵ȡ�������ѧ����ģ�������Ӿ���

function [ theta1,theta10,theta2,theta20] = Paden5( twist1,twist2,p,q)
%% ���봦��
if  isa(twist1,'Twist') && isa(twist2,'Twist')
%�ж������Ƿ����Twist���岢��ȡw�����߷��򣩺�r��������һ�㣩
    w1=twist1.w;
    r1=twist1.pole;
    w2=twist2.w;
    r2=twist2.pole;
    w1=w1(:);
    r1=r1(:);
    w2=w2(:);
    r2=r2(:);
elseif isa(twist1,'double') && isa(twist2,'double') && length(twist1)==6 ...
        && length(twist2)==6
%�ж������Ƿ������ά��������ȡw�����߷��򣩺�r��������һ�㣩
    w1=twist1(1:3)/(norm(twist1(1:3)));
    r1=twist1(4:6);
    w2=twist2(1:3)/(norm(twist2(1:3)));
    r2=twist2(4:6);    
    w1=w1(:);
    w2=w2(:);
    r1=r1(:);
    r2=r2(:);
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
u1=q-r1;
v1=p-r2;
%%��u��v�ڴ�ֱ��w�����ͶӰ
u=u1-(u1'*w1)*w1;
v=v1-(v1'*w1)*w1;
%���r1��r2�ڴ�ֱ��w�����ͶӰ
r1=q-u;
r2=p-v;
de1=norm(u);
de2=norm(v);
de3=norm(r1-r2);
m=(de1^2+de3^2-de2^2)/(2*de1*de3);
if 1<=m && m<=1+10^-5 %�ж��Ƿ��޽⣨����������
    m=1;
elseif -1-10^-5<=m && m<=-1
    m=-1;
end
if m>1+10^-5 || m<-1-10^-5
    theta1=nan;
    theta10=nan;
    theta2=nan;
    theta20=nan;
    return
end
alpha1=acos(m);
%���м�����z������p1
zx=cos(alpha1)*de1*(r2-r1)/de3;
zy=sin(alpha1)*de1*cross((r2-r1),w1)/norm(cross((r2-r1),w1));
z=zy+zx;
p1=r1+z;
theta2=Paden1(twist2,p,p1);
theta1=Paden1(twist1,p1,q);
%���м�����z0������p1=0
z0=zx-zy;
p0=r1+z0;
theta20=Paden1(twist2,p,p0);
theta10=Paden1(twist1,p0,q);
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