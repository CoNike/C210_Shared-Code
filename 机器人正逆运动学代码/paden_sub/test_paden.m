% Paden test
%用于测试指数积逆解中Paden子问题的使用

%关于指数积的内容可以参考一下文献：
%熊有伦等《机器人学：建模控制与视觉》
%李泽湘等《机器人学的几何基础》
%代码可以参考Peter Corke的RTB工具箱

%creator: Li Mingyue  Time:2019/9/30
%Huazhong University of Science and Technology
%% Paden1测试
% 测试输入报错
% w1=[0 1 0]';
% r1=[0 2 0]';
% tw1=Twist('R',w1,r1);
% r=[0 1 0]';
% p1=[1 2 -1 1]';
% q=[0 0 -1]';
% p=[0 0 0]';
% theta=Paden1(tw1,p1,q) %测试点的输入
% theta=Paden1(r,p,q) %测试旋量的输入

% 测试输出
% theta=Paden1(tw1,p,q) %测试无解时的输出为NaN

%随机测试输出：测试结果正确
% w2=[0 0 1]';
% r2=[0 0 2]';
% tw2=Twist('R',w2,r2);
% p=[1 0 0]';
% theta0=-pi+2*pi.*rand(10,1);
% for i=1:10
%     q2=[cos(theta0(i)),sin(theta0(i)),0]';
%     theta=Paden1(tw2,p,q2);
%     dis(i)=theta-theta0(i);
% end                           

%测试不同输入结果：测试结果正确
% w2=[0 0 1]';
% r2=[0 0 2]';
% p=[1 0 0]';
% q=[0 1 0]';
% tw2=Twist('R',w2,r2);
% tw3=[w2;r2];
% theta1=Paden1(tw2,p,q)
% theta2=Paden1(tw3,p,q)

%% Paden2测试
% 测试输入报错
% w1=[0 0 1]';
% r1=[0 0 2]';
% w2=[0 1 0]';
% r2=[0 2 0]';
% tw1=Twist('R',w1,r1);
% tw2=Twist('R',w1,r1);
% tw3=Twist('R',w2,r2);
% p=[1 0 0]';
% p1=[1 0 0 0]';
% q=[0 1 0]';
% r=[0 0 1]';
% [theta1,theta10,theta2,theta20]=Paden2( tw1,tw2,p1,q) 
% [theta1,theta10,theta2,theta20]=Paden2( tw1,tw3,p1,q,r) %测试点的输入
% [theta1,theta10,theta2,theta20]=Paden2( tw1,w1,p,q) 
% [theta1,theta10,theta2,theta20]=Paden2( tw3,w1,p,q,r) %测试旋量的输入

%测试输出
% [theta1,theta10,theta2,theta20]=Paden2( tw1,tw3,p,q,r) %测试无解时的输出为NaN

%随机测试输出：测试结果正确
% w1=[0,0,2]';
% w2=[0,2,0]';
% r1=[0,0,1]';
% r2=[0,1,0]';
% tw1=Twist('R',w1,r1);
% tw2=Twist('R',w2,r2);
% theta0=-pi+2*pi.*rand(10,1);
% for i=1:10
%     p=[cos(theta0(i)),sin(theta0(i)),1]';
%     q=[cos(theta0(i)),1,sin(theta0(i))]';
%     [theta1,theta10,theta2,theta20]=Paden2(tw1,tw2,p,q)
%     dis1(i)=theta1-theta2;
%     dis2(i)=theta10-theta20;
% end    

%测试不同输入结果：测试结果正确
% w1=[0,0,2]';
% w2=[0,2,0]';
% r1=[0,0,1]';
% r2=[0,1,0]';
% tw1=Twist('R',w1,r1);
% tw2=Twist('R',w2,r2);
% tw3=[w1;r1];
% tw4=[w2;r2];
% p=[1,0,1]';
% q=[1,1,0]';
% r=[0,0,0]';
% [theta1,theta10,theta2,theta20]=Paden2(tw1,tw2,p,q,r)
% [theta3,theta30,theta4,theta40]=Paden2(tw3,tw4,p,q,r)

%% Paden3测试
% 测试输入报错
% w1=[0 1 0]';
% r1=[0 2 0]';
% tw1=Twist('R',w1,r1);
% r=[0 1 0]';
% p1=[1 2 -1 1]';
% q=[2 0 0]';
% p=[-1 0 0]';
% De=20;
% theta=Paden3(tw1,p1,q,De) %测试点的输入
% theta=Paden3(r,p,q,De) %测试旋量的输入

% 测试输出
% [theta,theta1]=Paden3(tw1,p,q,De) %测试无解时的输出为NaN

%随机测试输出：测试结果正确
% w1=[0 0 1]';
% r1=[0 0 2]';
% tw1=Twist('R',w1,r1);
% q=[4 0 0]';
% De=5;
% theta0=-pi+2*pi.*rand(10,1);
% for i=1:10
%     p=[3*cos(theta0(i)),3*sin(theta0(i)),0]';
%     [theta,theta1]=Paden3(tw1,p,q,De);
%     dis(i)=abs(theta-theta1);
% end

%测试不同输入结果：测试正确
% w1=[0 1 0]';
% r1=[0 2 0]';
% tw1=Twist('R',w1,r1);
% tw2=[w1;r1];
% p=[-3,0,0]';
% q=[4 0 0]';
% De=5;
% [theta,theta1]=Paden3(tw1,p,q,De)
% [theta0,theta10]=Paden3(tw2,p,q,De)

%% Paden4测试
%测试输入报错
% w1=[0 0 1]';
% r1=[0 0 2]';
% w2=[0 1 0]';
% r2=[0 2 0]';
% tw1=Twist('R',w1,r1);
% tw2=Twist('R',w2,r2);
% p=[0 2 1]';
% p1=[1 0 0 0]';
% q=[0 1 0]';
% [theta1,theta10]=Paden4(tw1,tw2,p1,q) %测试点的输入
% [theta1,theta10]=Paden4(tw2,w1,p,q) %测试旋量的输入

%测试输出
% [theta1,theta10]=Paden4(tw1,tw2,p,q)%测试无解时的输出为NaN

% %随机测试输出：测试结果正确
% w1=[0 0 1]';
% r1=[0 0 2]';
% w2=[0 1 0]';
% r2=[0 2 0]';
% tw1=Twist('R',w1,r1);
% tw2=Twist('R',w2,r2);
% q=[0 1 0]';
% theta0=-pi+2*pi.*rand(10,1);
% for i=1:10
%     p=[cos(theta0(i)),1,sin(theta0(i))]';
%     [theta,theta1]=Paden4(tw1,tw2,p,q); %tw2与tw1平面只有一个交点q，因此结果为0
%     dis(i)=abs(theta-theta1);
% end

%测试不同输入：测试结果正确
% w1=[0 0 1]';
% r1=[0 0 2]';
% w2=[0 1 0]';
% r2=[0 2 0]';
% tw1=Twist('R',w1,r1);
% tw2=Twist('R',w2,r2);
% tw3=[w1;r1];
% tw4=[w2;r2];
% p=[0,1,1]';
% q=[0 2 0]';
% [theta,theta1]=Paden4(tw1,tw2,p,q)
% [theta0,theta10]=Paden4(tw3,tw4,p,q)

%% Paden5测试
%测试输入报错
% w1=[0 0 1]';
% r1=[0 0 2]';
% w2=[0 0 1]';
% r2=[0 2 2]';
% tw1=Twist('R',w1,r1);
% tw2=Twist('R',w2,r2);
% p=[0 2.5 0]';
% p1=[1 0 0 0]';
% q=[0 0.5 0]';
% [theta1,theta10,theta2,theta20]=Paden5(tw1,tw2,p1,q) %测试点的输入
% [theta1,theta10,theta2,theta20]=Paden5(tw2,w1,p,q) %测试旋量的输入

%测试输出
% [theta1,theta10,theta2,theta20]=Paden5(tw1,tw2,p,q)

%随机测试输出
% w1=[0 0 1]';
% r1=[0 0 2]';
% w2=[0 0 1]';
% r2=[0 2 2]';
% tw1=Twist('R',w1,r1);
% tw2=Twist('R',w2,r2);
% p=[0 3 0]';
% theta0=-pi+2*pi.*rand(10,1);
% for i=1:10
%     q=[cos(theta0(i)),sin(theta0(i)),0]';
%     [theta1,theta10,theta2,theta20]=Paden5(tw1,tw2,p,q); 
%     dis1(i)=theta1-theta10;
%     dis2(i)=theta2-theta20;
% end

%测试不同输入：测试结果正确
% w1=[0 0 1]';
% r1=[0 0 2]';
% w2=[0 0 1]';
% r2=[0 2 2]';
% tw1=Twist('R',w1,r1);
% tw2=Twist('R',w2,r2);
% tw3=[w1;r1];
% tw4=[w2;r2];
% p=[0 3 0]';
% q=[0 2 0]';
% [theta1,theta10,theta2,theta20]=Paden5(tw1,tw2,p,q);
% [theta3,theta30,theta4,theta40]=Paden5(tw3,tw4,p,q);

%% Padene测试
%测试输入报错
% w1=[0 0 1]';
% r1=[0 0 2]';
% w2=[0 1 0]';
% r2=[0 2 0]';
% tw1=Twist('R',w1,r1);
% tw2=Twist('R',w2,r2);
% p=[1 0 0]';
% p1=[1 0 0 0]';
% q1=[0 1 0]';
% q2=[0 2 0]';
% r0=[0 1 1]';
% de1=2;
% de2=1;
% [q01,q02,q03,q04]=Padene(tw1,tw2,p1,q1,q2,de1,de2)
% [q01,q02,q03,q04]=Padene(tw1,tw2,p1,q1,q2,r0,de1,de2)  %测试点的输入
% [q01,q02,q03,q04]=Padene(tw1,w2,p,q1,q2,de1,de2)
% [q01,q02,q03,q04]=Padene(tw1,r2,p,q1,q2,r0,de1,de2)  %测试旋量的输入

%测试输出
% [q01,q02,q03,q04]=Padene(tw1,tw2,p,q1,q2,r0,de1,de2) %测试无解时输出

%随机测试：测试结果正确
% w1=[0 0 1]';
% r1=[0 0 2]';
% w2=[0 1 0]';
% r2=[0 2 0]';
% tw1=Twist('R',w1,r1);
% tw2=Twist('R',w2,r2);
% q1=[0 1 2]';
% de1=1;
% q2=[0 2 1]';
% de2=1;
% r0=[0 0 0]';
% theta0=-pi+2*pi.*rand(10,1);
% for i=1:10
%     p=[2*cos(theta0(i)),2,2*sin(theta0(i))]';
%     [q01,q02,q03,q04]=Padene(tw1,tw2,p,q1,q2,r0,de1,de2);
% end

%测试不同输入
% w1=[0 0 1]';
% r1=[0 0 2]';
% w2=[0 1 0]';
% r2=[0 2 0]';
% tw1=Twist('R',w1,r1);
% tw2=Twist('R',w2,r2);
% tw3=[w1;r1];
% tw4=[w2;r2];
% q1=[0 1 2]';
% de1=1;
% q2=[0 2 1]';
% de2=1;
% r0=[0 0 0]';
% [q01,q02,q03,q04]=Padene(tw1,tw2,p,q1,q2,r0,de1,de2);
% [q05,q06,q07,q08]=Padene(tw3,tw4,p,q1,q2,r0,de1,de2);