% %auboi10 robot
d1=0.163;d4=0.2013;d5=0.1025;d6=0.094;
a2=0.647;a3=0.6005;

q=zeros(8,6);

L1= Link('revolute', 'd', d1,     'a', 0,        'alpha', pi/2,'offset',0);
L2= Link('revolute', 'd', 0,         'a', a2,    'alpha', pi,'offset',pi/2);
L3= Link('revolute', 'd', 0,         'a', a3,   'alpha', pi,'offset',0);
L4= Link('revolute', 'd', d4,    'a', 0,        'alpha', -pi/2,'offset',-pi/2);
L5= Link('revolute', 'd', d5,    'a', 0,        'alpha', pi/2,'offset',0);
L6= Link('revolute', 'd', d6,     'a', 0,        'alpha', 0,'offset',0);

% 
% qn=[-23.140575 -14.042009 109.599879 38.842449 9.966328 53.998950];
% T01=DHstd(90,0,d1,qn(1));
% T12=DHstd(180,a2,0,qn(2)+90);
% T23=DHstd(180,a3,0,qn(3));
% T34=DHstd(-90,0,d4,-90+qn(4));
% T45=DHstd(90,0,d5,qn(5));
% T56=DHstd(0,0,d6,qn(6));
% 
% T06=T01*T12*T23*T34*T45*T56
aubo=SerialLink([L1,L2,L3,L4,L5,L6]);
qz=[0  0 0 0 0 0];
qn=[-23.140575 -14.042009 109.599879 38.842449 9.966328 53.998950]*pi/180;
% qn=[0.1 0.2 0.3 0.4 0.5 0.6];

TG=aubo.fkine(qn);

% q=AuboIkinClose(TG,qn);
q=AuboIkinClose(TG);
% n=TG.n;
% o=TG.o;
% a=TG.a;
% p=TG.t;
% TG=TG.T;
% 
% m1=d6*a(2)-p(2);
% n1=d6*a(1)-p(1);
% x1(1)=atan2(m1,n1)-atan2(d4,sqrt(m1^2+n1^2-d4^2));
% x1(2)=atan2(m1,n1)-atan2(d4,-sqrt(m1^2+n1^2-d4^2));
% q(1:4,1)=x1(1);
% q(5:8,1)=x1(2);
% 
% 
% for i=1:2
%     x5(2*i-1)=acos(a(1)*sin(x1(i))-a(2)*cos(x1(i)));
%     x5(2*i)=-acos(a(1)*sin(x1(i))-a(2)*cos(x1(i)));
% end
% for i=1:4
%     m6=n(1)*sin(x1(floor((i+1)/2)))-n(2)*cos(x1(floor((i+1)/2)));
%     n6=o(1)*sin(x1(floor((i+1)/2)))-o(2)*cos(x1(floor((i+1)/2)));
%     x6(i)=atan2(m6,n6)-atan2(-sin(x5(i)),0);
%     q(2*i-1:2*i,5)=x5(i);
%     q(2*i-1:2*i,6)=x6(i);
% end
% 
% for i=1:8
%     m3=p(1)*cos(q(i,1))-d6*(a(1)*cos(q(i,1))+a(2)*sin(q(i,1)))-...
%         d5*(cos(q(i,6))*(o(1)*cos(q(i,1))+o(2)*sin(q(i,1)))+...
%         sin(q(i,6))*(n(1)*cos(q(i,1))+n(2)*sin(q(i,1))))+...
%         p(2)*sin(q(i,1));
%     n3=p(3)-d1-a(3)*d6-d5*(o(3)*cos(q(i,6))+n(3)*sin(q(i,6)));
%     if mod(i,2)==1
%         q(i,3)=acos((m3^2+n3^2-a2^2-a3^2)/(2*a2*a3));
%         if ~isreal(q(i,3))
%     q(i,:)=nan;        
%         else
%          s2=-((a2+a3*cos(q(i,3)))*m3-n3*a3*sin(q(i,3)))/...
%         (a2^2+a3^2+2*a2*a3*cos(q(i,3)));
%     c2=(m3+s2*(a2+a3*cos(q(i,3))))/(a3*sin(q(i,3)));
%     q(i,2)=atan2(s2,c2);
%     s234=-(cos(q(i,6))*(o(1)*cos(q(i,1))+o(2)*sin(q(i,1)))...
%         +sin(q(i,6))*(n(1)*cos(q(i,1))+n(2)*sin(q(i,1))));
%     c234=o(3)*cos(q(i,6))+n(3)*sin(q(i,6));
%     q(i,4)=atan2(s234,c234)-q(i,2)+q(i,3);
%         end
%     else
% 
%         q(i,3)=-acos((m3^2+n3^2-a2^2-a3^2)/(2*a2*a3));
%      if ~isreal(q(i,3))
%             q(i,:)=nan;
%     else
%          s2=-((a2+a3*cos(q(i,3)))*m3-n3*a3*sin(q(i,3)))/...
%         (a2^2+a3^2+2*a2*a3*cos(q(i,3)));
%     c2=(m3+s2*(a2+a3*cos(q(i,3))))/(a3*sin(q(i,3)));
%     q(i,2)=atan2(s2,c2);
%     s234=-(cos(q(i,6))*(o(1)*cos(q(i,1))+o(2)*sin(q(i,1)))...
%         +sin(q(i,6))*(n(1)*cos(q(i,1))+n(2)*sin(q(i,1))));
%     c234=o(3)*cos(q(i,6))+n(3)*sin(q(i,6));
%     q(i,4)=atan2(s234,c234)-q(i,2)+q(i,3);
%         end
%     end
% end

% 
%    
