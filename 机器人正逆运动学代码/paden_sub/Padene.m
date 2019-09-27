%Padene ������������p����������ཻ��ת�ᣨtwist2,twist1Ϊ������ߵ��˶�������
%����q1����de1��q2����Ϊde2�ĵ�
%r0Ϊ���������Ľ���
%���Ϊ������ܵĹؽڽǶ�
%����twist1��twist2Ϊ�����������ά����
%���Ϊ�ؽڽǣ����ȣ�

%�ο����ף�
%�����׵ȡ�������ѧ����ģ�������Ӿ���

function [ q01,q02,q03,q04 ] = Padene(twist1,twist2,p,q1,q2,r0,de1,de2)
%% ���봦��
if nargin == 7 
%�ж���������Ƿ��������r0�����ޣ������ɣ�ͬʱ�ж������Ƿ���ȷ
    r=twistcross(twist1,twist2); %����twistcross����������SE3�½���r0
    r0=r(1:3);                   %ת��Ϊ�ѿ����ռ�����ϵ�µ���ά������
end
%�ж������Ƿ����Twist��������Ƿ�Ϊ��ά����
if  isa(twist1,'Twist') && isa(twist2,'Twist')
elseif isa(twist1,'double') && isa(twist2,'double') && length(twist1)==6 ...
        && length(twist2)==6
else
    error('�����ʽ����')
end
if isa(p,'double') && isa(q1,'double') && isa(q2,'double') && isa(r0,'double') ... 
        && length(p)==3 && length(q1)==3 && length(q2)==3 && length(r0)==3
%�ж�������Ƿ����Ҫ��
    p=p(:);
    q1=q1(:);
    q2=q2(:);
    r0=r0(:);
else
    error('�����ʽ����')
end
%% ������
%���������Ľ���
w=p-r0;
u=q1-r0;
v=q2-r0;
de3=norm(w);
de4=norm(u);
de5=norm(v);
%��λ������
u1=u/(norm(u));
v1=v/(norm(v));

t1=de3*((de3^2+de4^2-de1^2)/(2*de3*de4));
t2=de3*((de3^2+de5^2-de2^2)/(2*de3*de5));
%���z=x1*u1+x2*v1+x3*cross(u1,v1)�е�ϵ��
x1=(t1-u1'*v1*t2)/(1-(u1'*v1)^2);
x2=(t2-u1'*v1*t1)/(1-(u1'*v1)^2);
x3_1=(de3^2-x1^2-x2^2-2*x1*x2*u1'*v1)/(norm(cross(u1,v1))^2);
if abs(x3_1-0)<0.01*min(abs(x1),abs(x2)) %�ж��Ƿ��޽⣨����������
    x3=0;
elseif x3_1>0
    x3=sqrt(x3_1);
else
    %     disp('there is no answer');
    q01=nan;
    q02=nan;
    q03=nan;
    q04=nan;
    return
end     
z=x1*u1+x2*v1+x3*cross(u1,v1);
q=z+r0;
[theta10,theta11,theta20,theta21]=Paden2(twist1,twist2,p,q,r0);
z1=x1*u1+x2*v1-x3*cross(u1,v1);
q1=z1+r0;
[theta12,theta13,theta22,theta23]=Paden2(twist1,twist2,p,q1,r0);
q01=[theta10,theta20];
q02=[theta11,theta21];   
q03=[theta12,theta22];  
q04=[theta13,theta23];   
end
