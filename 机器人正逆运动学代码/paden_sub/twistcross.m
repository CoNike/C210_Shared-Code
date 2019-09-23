%��������ת���ߵĽ���
%Ŀǰ�޷�����ƶ��ؽڣ�
%����twist Ϊ�����������ά������
%���Ϊ�������㡣
%example:
%w1=[0 0 1]';r1=[0 0 0]';
%w2=[1 0 0]';r2=[1 0 1]';
%twist1=Twist('R',w1,r1);
%twist2=Twist('R',w2,r2);
%p=twistcross(twist1,twist2)
%or
%twist1=[w1;r1];
%twist2=[w2;r2];
%p=twistcross(twist1,twist2)

function  p=twistcross(twist1,twist2)
%% ���봦�� 
if isa(twist1,'Twist') && isa(twist2,'Twist')
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
    w1=twist1(1:3);
    r1=twist1(4:6);
    w2=twist2(1:3);
    r2=twist2(4:6);
    w1=w1(:);
    r1=r1(:);
    w2=w2(:);
    r2=r2(:);
else
    error('�����ʽ����')
end

%% �ж������Ƿ���
%��������
v1=cross(r1,w1);
v2=cross(r2,w2);
if abs((w1'*v2+w2'*v1))>10^-5
    error('�����߲�����')
end

%% ��⽻��
if norm(cross(w1,w2))<10^-5
    %������ƽ��
    p=[w1;0];
else
    %�ཻ
    vec=cross(cross(w1,w2),w2);%����w2�Ҳ���w1��ֱ������
    lambda=(r2'*vec-r1'*vec)/(w1'*vec);
    p=lambda*w1+r1;
    p=[p;1];
end 
end