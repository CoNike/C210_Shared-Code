%COBOTTA 逆运动学测试
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

%% 工作空间外点检测
q0=[0.51 0.34 0.573 0.634 0.45 0.23];
T=SE3(0,1,1);
TG=T.T;
[qk2,n2,dis2]=cobotta_ik_num2(TG,q0);
%% 轨迹验证
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
%% 不同算法验证
% [qk2,n2,dis2]=cobotta_ik_num2(TG,q0);
% [qk1,n1,dis1]=cobotta_ik_num(TG,q0);
%% beta最佳值预测
%后期beta最佳值
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

%前期beta最佳值
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

 %验证选择的转换距离
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
%% 一些验证

% q0=[0.1 0.2 0.3 0.4 0.5 0.6];
% qg=[1.4 1.5 1.2 0.5 1.7 1.8];
% q=jtraj(q0,qg,10);
% [rowq,colq]=size(q);
% for i=1:rowq
%     TG=cobotta.fkine(q(i,:));
%     T0=cobotta.fkine(q0); %迭代初始角度对应的空间位置
%     T=TG*inv(T0); %求初始位置到目标位置的变换
%     v=T.t;         %求出变刚体变换对应的平移量
%     w=vex(T.t2r-eye(3)); %将机器人旋转矩阵转换为轴角的乘积
%     delta=[v;w];        %合并为6维矢量
%     dis1=norm(delta);    %求delta模长作为判断值
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
