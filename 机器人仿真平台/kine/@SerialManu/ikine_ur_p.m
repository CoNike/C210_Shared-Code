%����ָ������������URϵ�л����˵����˶�ѧ
% qk=ikine_ur_p(robot,Tg,q0)
%robot��Ҫ����234�ؽ�ƽ�У�5,6�ཻ
%����ģ�͵ĸ�ʽΪSelialManu
%����TgΪĿ��λ�˾���
%q0Ϊ�ο��Ƕȣ�����ʱ���ֻ��һ���

%��������㷨���Բο��ĵ�
function qk=ikine_ur_p(robot,Tg,q0)
%% ȷ�������ʽ
if isa(Tg,'SE3')
    Tg=Tg.T;
end
%% ��ȡ��������ز���
r=robot.r;   %���ؽ����߾����ĵ�
w=robot.w;   %�ؽ����߷���
link=robot.jolinks; %��ȡ�������
T0=robot.T0; %������ĩ�˳�ʼλ��
%�������ߵ����������ڿ���ɾ��
twist1=Twist('R',w(1,:),r(1,:));
twist2=Twist('R',w(2,:),r(2,:));
twist3=Twist('R',w(3,:),r(3,:));
twist4=Twist('R',w(4,:),r(4,:));
twist5=Twist('R',w(5,:),r(5,:));
twist6=Twist('R',w(6,:),r(6,:));
%% ������
%�Ƕ�1���
Ta=Tg/T0;
p56=twistcross(twist5,twist6);
p_56=Ta*p56;
[x11,x12]=paden4(twist1,twist2,p56(1:3),p_56(1:3));
qk(1:4,:)=x11;
qk(5:8,:)=x12;

%�Ƕ�5,6���
for i=1:2
    q1=qk(4*i,1);
    T1_inv=link(1).isom(-q1);
    Tb=T1_inv*Ta;
    w2=w(2,:);
    w2=w2(:);
    vec=inv(Tb)*[w2;0];
    r0=[0 0 0]';
    twist51=Twist('R',w(5,:),r0);
    twist61=Twist('R',w(6,:),r0);
    [x51,x52,x61,x62]=Paden2(twist51,twist61,vec(1:3),w2(1:3),r0);
    qk(i*4-3:i*4-2,5)=x51;
    qk(i*4-1:i*4,5)=x52;
    qk(i*4-3:i*4-2,6)=x61;
    qk(i*4-1:i*4,6)=x62;
end
% �Ƕ�234���
for i=1:4
    q1=qk(2*i,1);
    T1_inv=link(1).isom(-q1);
    q5=qk(2*i,5);
    T5_inv=link(5).isom(-q5);
    q6=qk(2*i,6);
    T6_inv=link(6).isom(-q6);
    Tc=T1_inv*Ta*T6_inv*T5_inv;
    p4=r(4,:)';
    p40=Tc*[p4;1];
    [x21,x22,x31,x32]=Paden5(twist2,twist3,p4,p40(1:3));
    T2_inv1=link(2).isom(-x21);
    T2_inv2=link(2).isom(-x22);
    T3_inv1=link(3).isom(-x31);
    T3_inv2=link(3).isom(-x32);
    Td1=T3_inv1*T2_inv1*Tc;
    p_rand=[pi sqrt(2) 3]';
    p=Td1*[p_rand;1];
    x41=Paden1(twist4,p_rand,p(1:3));
    Td2=T3_inv2*T2_inv2*Tc;
    p_rand=[pi sqrt(2) 3]';
    p=Td2*[p_rand;1];
    x42=Paden1(twist4,p_rand,p(1:3));
    qk(2*i,2)=x21;
    qk(2*i-1,2)=x22;
    qk(2*i,3)=x31;
    qk(2*i-1,3)=x32;
    qk(2*i,4)=x41;
    qk(2*i-1,4)=x42;
end
qk=qk(~any(isnan(qk),2),:); %�ų������н�

%�Ӱ������ѡȡ��ο����������
if nargin>2
    if length(q0)~=robot.n
        error('����ο�������')
    end
    n=size(qk,1);
    for i=1:n
        dis(i)=norm(qk(i,:)-q0);
    end
    [~,k]=min(dis);
    qtemp=qk(k,:);
    qk=qtemp;
    
end
end