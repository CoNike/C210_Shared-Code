%轨迹算法验证

%% tracj_5验证
%特定点验证
% q0=[0 0 0 0 0 0];
% qf=[1 1 1 1 1 1];
% [q1,qd1,qdd1]=jtraj(q0,qf,50);
% [q2,qd2,qdd2]=traj_5(q0,qf,50);
% t=0:0.01:0.49;
% [q2,qd2,qdd2]=traj_5(q0,qf,t);

% q0=[1 0.4 3 1 0.8 2];
% qf=[0 3 2.3 1.2 2.2 3];
% [q1,qd1,qdd1]=jtraj(q0,qf,50);
% [q2,qd2,qdd2]=Jtraj_h(q0,qf,50);

%% traj_3
% q0=[0 0 0 0 0 0];
% qf=[1 2 1 1 1 1];
% qm=[0.5 1 0.5 0.5 0.5 0.5];
% % [q1,qd1,qdd1]=tracj_t(q0,qf,3,'V',qm);
% [q1,qd1,qdd1]=tracj_t(q0,qf,3,'A',qm);

%% function [q,qd,qdd]=jtraj_par(q0,td,qm)
% q0=[0 2 5 3 4 6 0]';
% td=[2,4,2,1,3,6]';
% qm=2;
% [q,qd,qdd]=traj_par(q0,td,qm);

%% function T=tracj_l(T0,Tg,t)
% mdl_puma560;
% T0=p560.fkine([0 0 0 0 0 0]);
% T0=T0.T;
% TG=p560.fkine([0 -pi/2 0 0 pi/2 pi/2]);
% TG=TG.T;
% T1=tracj_l(T0,TG,50);
% T2=tracj_l(T0,TG,50,'eul');
% T3=tracj_l(T0,TG,50,'angvec');

%% function T=tracj_cir(T1,T2,T3,t)
% p1=[0 0.5 1.3]';
% p2=[0.7 0.3 1  ]';
% p3=[0.6 0.7 0.8]';
% p0=[0 0 0];
% [p0,r]=circle_r(p1,p2,p3);
% p=circle_jtraj(p1,p2,p3,50);
% for i=1:size(p,1)
%     plot3(p(i,1),p(i,2),p(i,3),'r');
%     hold on;
%     arrow3(p(i,:),p0','g')
% end
% T1=SE3(p1);
% T1=T1.T;
% T2=SE3(p2);
% T2=T2.T;
% T3=SE3(p3);
% T3=T3.T;
% T=tracj_cir(T1,T2,T3,50);
%   plot3(T(:,4),T(:,5),T(:,6),'r');
%       hold on;
% for i=1:size(T,1)
%     Tg=transl(T(i,4),T(i,5),T(i,6))*rpy2tr(T(i,1:3));
%     q(i,:)=UR10ikineClose(Tg);
%     ur10.plot(q(i,:));
% end

%指定目标点绘制三维位姿
% p1=[0 0.5 1.3]';
% p2=[0.7 0.3 1  ]';
% p3=[0.6 0.7 0.8]';
% p0=[0 0 0];
% T1=circle_pose(p1,p2,p3,p0,50);
% for i=1:50
%     plot(SE3(T1{i}),'r')
%     hold on;
% end
%指定初始位姿绘图
% p1=[0 0.5 1.3]';
% p2=[0.7 0.3 1  ]';
% p3=[0.6 0.7 0.8]';
% p0=[ 0 0 0]';
% n=nor_vec_p(p1,p2,p3);
% z0=(p1-p0)/norm(p1-p0);
% p1p2=p2-p1;
% x0=cross(p1p2,z0)/norm(cross(p1p2,z0));
% y0=cross(z0,x0);
% R=[x0 y0 z0];
% T1=circle_pose2(p1,p2,p3,p0,50,R);
% for i=1:50
%     plot(SE3(T1{i}),'r')
%     hold on;
% end
% hold on;
% plot3(p0(1),p0(1),p0(2),'r*');
%%  function [q,qd,qdd]=tracj_t(q0,qf,tf,T,qm)
% q0=[0 0 0 0 0 0];
% qf=[1 2 3 4 5 6];
% tf=3;
% qm=[1 2 3 4 5 6];
% [q,qd,qdd]=tracj_t(q0,qf,tf,'A',qm)