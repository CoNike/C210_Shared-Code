%COBOTTA ���˶�ѧ����
clc;
clear;
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

% q0=[0.51 0.34 0.573 0.634 0.45 0.23];
% qt=[0.45 0.37 0.453 0.643 0.38 0.24] ;
% TG=cobotta.fkine(qt);
% TG=TG.T;

%% �����ռ������
q0=[0.51 0.34 0.573 0.634 0.45 0.23];
T=SE3(0,1,1);
TG=T.T;
[qk2,n2,dis2]=cobotta_ik_num2(TG,q0);
%% �켣��֤
% q=jtraj(q0,qt,100);
% [rowq,colq]=size(q);
% for i=1:rowq
%     TG=cobotta.fkine(q(i,:));
%     TG=TG.T;
%     if i==1
%         [qk(i,:),n]=cobotta_ik_num(TG,q0);
%     else
%         [qk(i,:),n]=cobotta_ik_num(TG,qk(i-1,:));     
%     end
%     diedai(i)=n;
% end
% TG=cobotta.fkine(q(79,:));
% TG=TG.T;
% [qk,n]=cobotta_ik_num(TG,q(78,:));
%% ��ͬ�㷨��֤
% [qk2,n2,dis2]=cobotta_ik_num2(TG,q0);
% [qk1,n1,dis1]=cobotta_ik_num(TG,q0);
%% beta���ֵԤ��
%����beta���ֵ
%  beta=0.8:0.01:1.2;
%  for i=1:length(beta)
%      [~,n,~]=cobotta_ik_num3(TG,q0,beta(i));
%      diedai(i)=n;
%  end
%  
%   beta=0.85:0.001:1.04;
%  for i=1:length(beta)
%      [~,n,~]=cobotta_ik_num3(TG,q0,beta(i));
%      diedai(i)=n;
%  end
%  beta=0.9130~0.9499

%ǰ��beta���ֵ
% q0=[0.1 0.2 0.3 0.4 0.5 0.6];
% qg=[0.4 0.5 0.2 0.5 0.7 0.8];
% TG=cobotta.fkine(qg);
% TG=TG.T;
% %  beta=0.05:0.01:0.5;
% %  beta=0.19:0.005:0.32;
%  beta=0.25:0.001:0.31;
%  for i=1:length(beta)
%      [~,n,~]=cobotta_ik_num3(TG,q0,beta(i));
%      diedai(i)=n;
%  end
%  plot(diedai)
%  beta=0.25-0.31

 %��֤ѡ���ת������
%  q0=[0.1 0.2 0.3 0.4 0.5 0.6];
% qg=[1.4 1.5 1.2 1.5 1.7 1.8];
% TG=cobotta.fkine(qg);
% TG=TG.T;
%  beta=0.01:0.01:0.8;
% %  beta=0.19:0.005:0.32;
% %  beta=0.25:0.001:0.31;
%  for i=1:length(beta)
%      [~,n,~]=cobotta_ik_num3(TG,q0,beta(i));
%      diedai(i)=n;
%  end
%  plot(diedai)
%  todelta
%  [qk1,n1,dis1]=cobotta_ik_num2(TG,q0);
%% һЩ��֤

% q0=[0.1 0.2 0.3 0.4 0.5 0.6];
% qg=[1.4 1.5 1.2 0.5 1.7 1.8];
% q=jtraj(q0,qg,10);
% [rowq,colq]=size(q);
% for i=1:rowq
%     TG=cobotta.fkine(q(i,:));
%     T0=cobotta.fkine(q0); %������ʼ�Ƕȶ�Ӧ�Ŀռ�λ��
%     T=TG*inv(T0); %���ʼλ�õ�Ŀ��λ�õı任
%     v=T.t;         %��������任��Ӧ��ƽ����
%     w=vex(T.t2r-eye(3)); %����������ת����ת��Ϊ��ǵĳ˻�
%     delta=[v;w];        %�ϲ�Ϊ6άʸ��
%     dis1=norm(delta);    %��deltaģ����Ϊ�ж�ֵ
%     dis(i)=dis1;
%     TG=TG.T;
%     if i==1
%         [qk1(i,:),n1]=cobotta_ik_num(TG,q0);
%          [qk2(i,:),n2]=cobotta_ik_num2(TG,q0);
%     else
%         [qk1(i,:),n1]=cobotta_ik_num(TG,qk1(i-1,:));  
%         [qk2(i,:),n2]=cobotta_ik_num2(TG,qk2(i-1,:)); 
%     end
%     diedai1(i)=n1;
%     diedai2(i)=n2;
% 
% end
%     plot(diedai1,'r')
%     hold on;
%     plot(diedai2,'b')
