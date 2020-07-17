%seriamanu test
%用于测试JoLink、SerialManu的创建和函数调用发
%% 连杆创建
%创建空的连杆对象，会依据默认参数创建
% L1=JoLink();

%依据参数创建连杆
% w=[0 1 1];
% r=[1 2 3];
% L2=JoLink('w',w,'r',r);
% v=[1 2 3];
% L3=JoLink('v',v);

%复制连杆
% L4=JoLink(L2);

%% 创建SerialManu(串联机器人对象)
%创建空的机器人；
% R1=SerialManu();

%依据参数创建机器人
% L=[L2 L3 L4];
% T0=eye(4);
% R2=SerialManu(L,'T0',T0);

%复制机器人
% R3=SerialManu(R2);

%将DH参数创建的机器人转换为指数积的模型
% mdl_puma560;
% p560p=dh2poe(p560);

%利用XML文件创建机器人对象
%XML文件的格式参照puma560.xml
% p560x=xml2robot('puma560.xml');
%% 正运动学测试
% mdl_puma560;
% p560p=dh2poe(p560);
% q=zeros(1,6);

% 测量角度旋向是否正确
% for i=1:6
%     q(i)=1;
%     Tp=p560p.fkinep(q);
%     T=double(p560.fkine(q));
%     T-Tp
% end

%随机点,判断正解是否正确
% q1=rand(10,1)*2*pi-pi;
% q2=rand(10,1)*2*pi-pi;
% q3=rand(10,1)*2*pi-pi;
% q4=rand(10,1)*2*pi-pi;
% q5=rand(10,1)*2*pi-pi;
% q6=rand(10,1)*2*pi-pi;
% 
% for i=1:10
%     q=[q1(i) q2(i) q3(i) q4(i) q5(i) q6(i)];
%     Tp=p560p.fkinep(q);
%     T=double(p560.fkine(q));
%     T-Tp
% end

%% 雅可比矩阵测试
%两种雅克比测试
% J1= p560.jacob0(qn);
% J2=p560p.jacobp(qn,'d');
% 
% J1= p560.jacobe(qn);
% J2=p560p.jacobp(qn,'t');
%
%随机点测试
% q1=rand(10,1)*2*pi-pi;
% q2=rand(10,1)*2*pi-pi;
% q3=rand(10,1)*2*pi-pi;
% q4=rand(10,1)*2*pi-pi;
% q5=rand(10,1)*2*pi-pi;
% q6=rand(10,1)*2*pi-pi;
% for i=1:10
%     q=[q1(i) q2(i) q3(i) q4(i) q5(i) q6(i)];
%     J1= p560.jacob0(q);
%     J2=p560p.jacobp(q,'d');
%     J1-J2
%     J3= p560.jacobe(q);
%     J4=p560p.jacobp(q,'b');
%     J3-J4
% end


%% 数值迭代解逆解
%随机点测试
% q1=rand(10,1)*2*pi-pi;
% q2=rand(10,1)*2*pi-pi;
% q3=rand(10,1)*2*pi-pi;
% q4=rand(10,1)*2*pi-pi;
% q5=rand(10,1)*2*pi-pi;
% q6=rand(10,1)*2*pi-pi;
% for i=1:10
%     q=[q1(i) q2(i) q3(i) q4(i) q5(i) q6(i)];
%     q0=q+rand(6,1)'/20;
%     Tg=p560p.fkinep(q);
%     qk=p560p.ikine_num_p(Tg,q0);
%     T=p560p.fkinep(qk)-Tg
% end

%% ur逆解测试
%特定位置八组解验证
% mdl_ur5;
% UR5=dh2poe(ur5);
% T=UR5.fkinep([0.8 1.2 1.8 2.3 0.9 2.1]);
% qk=UR5.ikine_ur_p(T);
% for i=1:size(qk,1)
%     TG=UR5.fkinep(qk(i,:));
%     dis(i)=norm(TG(1:3,4)-T(1:3,4));
% end
% dis
%位置误差在10^-16量级

%随机角度一组解验证
%位置误差在10^-15量级
% mdl_ur5;
% UR5=dh2poe(ur5);
% q1=rand(10,1)*2*pi-pi;
% q2=rand(10,1)*2*pi-pi;
% q3=rand(10,1)*2*pi-pi;
% q4=rand(10,1)*2*pi-pi;
% q5=rand(10,1)*2*pi-pi;
% q6=rand(10,1)*2*pi-pi;
% for i=1:10
%     q=[q1(i) q2(i) q3(i) q4(i) q5(i) q6(i)];
%     q0=q+rand(6,1)'/200;
%     Tg=UR5.fkinep(q);
%     qk=UR5.ikine_ur_p(Tg,q0);
%     TG=UR5.fkinep(qk);
%     dis1(i)=norm(qk-q);
%     dis2(i)=norm(TG(1:3,4)-Tg(1:3,4));
% end
