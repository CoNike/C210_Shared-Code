% cobotta
function [q,n,dis]=cobotta_ik_num3(TG,q0,beta)

%�ж��Ƿ����õ�����㣬��û�����ã���Ĭ��Ϊ[0 0 0 0 0 0]
if nargin==1
    q0=[0 0 0 0 0 0];
end

%����cobotta��ز���
d1=0.18;
a2=0.165;
a3=-0.012;
d3=0.02;
d4=0.1775;
d5=-0.0645;
d6=0.045;
L1= Link('revolute', 'd', d1,     'a', 0,        'alpha', -pi/2,'offset',0);
L2= Link('revolute', 'd', 0,         'a', a2,    'alpha', 0,'offset',-pi/2);
L3= Link('revolute', 'd', d3,         'a', a3,   'alpha', pi/2,'offset',pi/2);
L4= Link('revolute', 'd', d4,    'a', 0,        'alpha', -pi/2,'offset',0);
L5= Link('revolute', 'd', d5,    'a', 0,        'alpha', pi/2,'offset',0);
L6= Link('revolute', 'd', d6,     'a', 0,        'alpha', 0,'offset',0);
cobotta=SerialLink([L1,L2,L3,L4,L5,L6]);

TG=SE3(TG);
%���õ�����ضԵĲ���
dis1=0;
dis2=0;%Ŀ������������������ĳ�ʼ����
n=2;
flag=1;%�����ж��Ƿ�ѭ���ı�־
% beta=1;
lambda=0.01;%���ñ�������λ�õĲ���
while flag==1
    T0=cobotta.fkine(q0); %������ʼ�Ƕȶ�Ӧ�Ŀռ�λ��
    T=TG*inv(T0); %���ʼλ�õ�Ŀ��λ�õı任
    v=T.t;         %��������任��Ӧ��ƽ����
    w=vex(T.t2r-eye(3)); %����������ת����ת��Ϊ��ǵĳ˻�
    delta=[v;w];        %�ϲ�Ϊ6άʸ��
    dis1=norm(delta);    %��deltaģ����Ϊ�ж�ֵ
    jaco=cobotta.jacob0(q0);  %��������q0ʱ���ſ˱Ⱦ���
    det_j=abs(det(jaco));   %����ſ˱�����ʽ�ľ���ֵ
    %����det_j��ȡ��ͬ�ĵ�������
    if  det_j>10^-4
        temp=inv(jaco)*delta;
        alpha=1;
    else
        %����λ�ø������õķ���
        lambda=0.01;
        temp=jaco'*inv(jaco*jaco'+lambda*eye(6))*delta;
        alpha=1.5;
    end
    if dis1>0.1
        beta=beta;
    else
        beta=0.92;
    end
    q0=q0+alpha*temp'*beta;          %�����º�ĽǶ�
    for i=length(q0)
        if q0(i)>=pi
            q0(i)=q0(i)-2*pi;
        elseif q0(i)<-pi
            q0(i)=q0(i)+2*pi;
        end
            
    end
    if ((abs(dis2-dis1))<10^-6)||(n>1000)        %�ж��Ƿ��������
        flag=0;
    end
    dis2=dis1;
    dis(n)=dis2;
    n=n+1;
end
q=q0;
% plot(dis)
end