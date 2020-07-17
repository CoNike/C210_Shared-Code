
function theta=UR10ikineClose(TG1, q0)
% DH²ÎÊý
%   alphq   a       d           theta 
%0  0       0       0           0
%1  pi/2    0       0.128       0
%2  0       -0.612  0           0
%3  0       -0.5723 0           0
%4  pi/2    0       0.163941    0
%5  -pi/2   0       0.1157      0
%6  0       0       0.0922      0
if nargin==1
    q0=[0 0 0 0 0 0];
end
TG1=SE3(TG1);
n=TG1.n;
o=TG1.o;
a=TG1.a;
p=TG1.t;
TG=TG1.T;
urdh=[0      0       0           0;
     pi/2    0       0.1273       0;
     0      -0.612   0           0;
     0      -0.5723  0           0;
     pi/2    0       0.163941    0;
     -pi/2   0       0.1157      0;
     0       0       0.0922     0];
 d=urdh(2:7,3);
 A=urdh(2:7,2);
theta=zeros(1,6);
m1=TG(2,3)*d(6)-TG(2,4);
n1=TG(1,3)*d(6)-TG(1,4);

x11=atan2(m1,n1)-atan2(d(4),sqrt(m1^2+n1^2-d(4)^2));
x12=atan2(m1,n1)-atan2(d(4),-sqrt(m1^2+n1^2-d(4)^2));
if norm(x11-q0(1))<=norm(x12-q0(1))
    theta(1)=x11;
else
    theta(1)=x12;
end

x51=acos(TG(1,3)*sin(theta(1))-TG(2,3)*cos(theta(1)));
x52=-acos(TG(1,3)*sin(theta(1))-TG(2,3)*cos(theta(1)));
if norm(x51-q0(5))<=norm(x52-q0(5))
    theta(5)=x51;
else
    theta(5)=x52;
end 

m6=TG(1,1)*sin(theta(1))-TG(2,1)*cos(theta(1));
n6=TG(1,2)*sin(theta(1))-TG(2,2)*cos(theta(1));
x61=atan2(m6,n6)-atan2(sin(theta(5)),0);
theta(6)=x61;

m3=d(5)*(sin(theta(6))*(n(1)*cos(theta(1))+n(2)*sin(theta(1)))+...
    cos(theta(6))*(o(1)*cos(theta(1))+o(2)*sin(theta(1))))-d(6)*...
    (a(1)*cos(theta(1))+a(2)*sin(theta(1)))+...
    p(1)*cos(theta(1))+p(2)*sin(theta(1));
n3=p(3)-d(1)-a(3)*d(6)+d(5)*(o(3)*cos(theta(6))+n(3)*sin(theta(6)));
x31=acos((m3^2+n3^2-A(2)^2-A(3)^2)/(2*A(2)*A(3)));
x32=-x31;

if norm(x31-q0(3))<=norm(x32-q0(3))
    theta(3)=x31;
else
    theta(3)=x32;
end 

s2=((A(3)*cos(theta(3))+A(2))*n3-A(3)*sin(theta(3))*m3)/...
    (A(2)^2+A(3)^2+2*A(2)*A(3)*cos(theta(3)));
c2=(m3+A(3)*sin(theta(3))*s2)/(A(3)*cos(theta(3))+A(2));
x21=atan2(s2,c2);
theta(2)=x21;

m4=-sin(theta(6))*(n(1)*cos(theta(1))+n(2)*sin(theta(1)))...
    -cos(theta(6))*(o(1)*cos(theta(1))+o(2)*sin(theta(1)));
n4=o(3)*cos(theta(6))+n(3)*sin(theta(6));
x4=atan2(m4,n4)-theta(2)-theta(3);
theta(4)=x4;


function T=dhstdr(alpha,a,d,theta)
    T1=SE3.Rz(theta)*SE3(0,0,d);
    T2=SE3.Rx(alpha)*SE3(a,0,0);
    T=T1*T2;
    T=T.T;
end
end
