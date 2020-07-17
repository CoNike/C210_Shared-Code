function q=abb660_ikine(TG,q0)
% abb660 逆运动学
%相关参数
 a1=0.3;a2=1.28;a3=1.35;a4=0.26;
 d1=0.8;d5=0.247;
TG=SE3(TG);
n=TG.n;
p=TG.t;
q=zeros(4,5);
%求解关节角4
s1=p(2);
c1=p(1);
q1_1=atan2(s1,c1);
q1_2=atan2(-s1,-c1);

q(1:2)=q1_1;
q(3:4)=q1_2;
for i=1:4          
    l1=p(3)-d1+d5;
    if abs(sin(q(i,1)))>0.1
        l2=p(2)/sin(q(i,1))-a1-a4;
    else
        l2=p(1)/cos(q(i,1))-a1-a4;
    end
    s23=(l1^2+l2^2-a2^2-a3^2)/(2*a2*a3);
    k23=1-s23^2;
    if k23<-10^-8
        q(i,5)=nan;
    else
        q23_1=atan2(s23,sqrt(k23));
        q23_2=atan2(s23,-sqrt(k23));
        if mod(i,2)==1
            q(i,5)=q23_1;
        else
            q(i,5)=q23_2;
        end
    end

    c3=(l2*(a3+a2*sin(q(i,5)))+a2*cos(q(i,5))*l1)/((a3+a2*sin(q(i,5)))^2+(a2*cos(q(i,5)))^2);
    s3=(l2-c3*(a3+a2*sin(q(i,5))))/(a2*cos(q(i,5)));
    q3=atan2(s3,c3);
    q(i,3)=q3;
    q2=q3+q(i,5);
    q(i,2)=q2;
    
    %判断初始求解的关节1是否存在问题
    k1=a3*cos(q(i,3))+a2*sin(q(i,2))+a1+a4;
    q1=atan2(p(2)/k1,p(1)/k1);
    if abs(q(i,1)-q1)>10^-4
        q(i,1)=nan;
    end
    q14=atan2(n(2),n(1));
    q4=q(i,1)-q14;
    q(i,4)=q4;
end

q=q(~any(isnan(q),2),:);
q=q(:,1:4);

[qrow,qcol]=size(q);
for  i=1:qrow
    for j=1:qcol
        if q(i,j)>pi
            q(i,j)=q(i,j)-2*pi;
        elseif q(i,j)<=-pi
            q(i,j)=q(i,j)+2*pi;
        end
    end
end
if nargin==2
    [j,~]=size(q);
    for i=1:j
        dis(i)=norm(q(i,:)-q0);  
    end
    [~,b2]=min(dis);
    q=q(b2,:);
end
end
%% 推导过程
% syms a1 a2 a3 a4 real;
% syms d1 d5 real;
% syms q1 q2 q3 q4 real;
% syms nx ny nz ox oy oz ax ay az px py pz real;
% % T01=DHstd_sy(-pi/2,a1,d1,q1);
% % T12=DHstd_sy(0,a2,0,q2-pi/2);
% % T23=DHstd_sy(0,a3,0,-q2+pi/2+q3);
% % T34=DHstd_sy(-pi/2,a4,0,-q3);
% % T45=DHstd_sy(0,0,d5,q4);
% 
% %   a1=0.3;a2=1.28;a3=1.35;a4=0.26;
% %  d1=0.8;d5=0.247;
% % 
% %  q1=0.3;q2=0.4;q3=0.5;q4=0.6;
%  TG=[nx ox ax px ;
%      ny oy ay py;
%      nz oz az pz;
%      0 0  0 1;];
%  
% T01=[ cos(q1), 0, -1.0*sin(q1), a1*cos(q1);
%     sin(q1),  0,      cos(q1), a1*sin(q1);
%     0,       -1.0,     0,         d1;
%     0,        0,            0,        1.0];
% 
% T12=[  sin(q2), cos(q2), 0,  a2*sin(q2);
%     -cos(q2), sin(q2), 0, -a2*cos(q2);
%     0,       0, 1,           0;
%     0,       0, 0,           1];
% 
% T23=[ sin(q2 - q3), -cos(q2 - q3), 0, a3*sin(q2 - q3);
%     cos(q2 - q3),  sin(q2 - q3), 0, a3*cos(q2 - q3);
%     0,             0, 1,               0;
%     0,             0, 0,               1];
% % 
% T34=[      cos(q3), 0, sin(q3),      a4*cos(q3);
% -1.0*sin(q3), 0, cos(q3), -1.0*a4*sin(q3);
%            0,            -1.0,0,               0;
%             0,               0,       0,             1.0];
% %  
%  T45=[ cos(q4), -sin(q4), 0,  0;
% sin(q4),  cos(q4), 0,  0;
%       0,        0, 1, d5;
%       0,        0, 0,  1];
%   
%  T02=simplify(T01*T12);
%  T03=simplify(T02*T23);
%  T04=simplify(T03*T34);
%  T05=simplify(T04*T45);
%  
% T15=simplify(T12*T23*T34*T45);
% T150=simplify(inv(T01)*TG); 
% T040=simplify(TG*inv(T45)); 