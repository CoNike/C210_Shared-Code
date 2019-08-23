**直接在Matlab里运行即可**

```c
/*   C210
*    @Copyright Huang zhouzhou & QinLiang
*    2019 /8 
*/
clc;
clear all;

L1 = Link('d', 0.089159, 'a', 0, 'alpha', pi/2);    
L2 = Link('d', 0, 'a', -0.425, 'alpha',0); 
L3 = Link('d', 0, 'a', -0.39225, 'alpha',0);
L4 = Link('d',0.10915, 'a', 0, 'alpha', pi/2);
L5 = Link('d', 0.09465, 'a', 0, 'alpha', -pi/2);
L6 = Link('d', 0.0823, 'a', 0, 'alpha',0);

robot=SerialLink([L1,L2,L3,L4,L5,L6]);
robot.name = 'UR5 robot';
%q1 = [90 -90 0 -90 -90 0] / 180 *pi;
q2 = [1.1498   -2.7874    1.5561   -1.9103   -1.1498   -0.0000];
%q1=[90.53 -88.03 62.88 -131.74 -90 -135.14] / 180 *pi;

result = robot.fkine(q2);
TG1 = result.T;

a2=-0.425;
a3=-0.39225;
d1=0.089159;
d4=0.10915;
d5=0.09465;
d6=0.0823 ;

n=TG1(1:3,1);
o=TG1(1:3,2);
a=TG1(1:3,3);
p=TG1(1:3,4);

%预设输出
 q=zeros(8,6);
%求解角度1
m1=d6*a(2)-p(2);
n1=d6*a(1)-p(1);
k = m1*m1+n1*n1-d4*d4;
if (k > -1e-8 && k < 0)
    k = 0;
end
q(1:4,1)=atan2(m1,n1)-atan2(d4,sqrt(k));
q(5:8,1)=atan2(m1,n1)-atan2(d4,-sqrt(k));
%求解角度5
for i=1:4
    q5=acos(a(1)*sin(q(2*i,1))-a(2)*cos(q(2*i,1)));
    if mod(i,2)==1
        q(2*i-1:2*i,5)=q5;
    else
        q(2*i-1:2*i,5)=-q5;
    end
end
%求解角度234
for i=1:8
    m6=n(1)*sin(q(i,1))-n(2)*cos(q(i,1));
    n6=o(1)*sin(q(i,1))-o(2)*cos(q(i,1));
    q(i,6)=atan2(m6,n6)-atan2(sin(q(i,5)),0);
    m3=d5*(sin(q(i,6))*(n(1)*cos(q(i,1))+n(2)*sin(q(i,1)))...
          +cos(q(i,6))*(o(1)*cos(q(i,1))+o(2)*sin(q(i,1))))...
        +p(1)*cos(q(i,1))+p(2)*sin(q(i,1))-d6*(a(1)*cos(q(i,1))+a(2)*sin(q(i,1)));
        
    n3=p(3)-d1-a(3)*d6+d5*(o(3)*cos(q(i,6))+n(3)*sin(q(i,6)));
    k3=(m3*m3+n3*n3-a2*a2-a3*a3)/(2*a2*a3);
    if (k3-1)>10^-6||(k3+1)<10^-6
       q3=nan;
    elseif (k3-1)>0 && (k3-1)<=10^-6
        q3=0;
    elseif (k3+1)<=0 && (k3+1)>10^-6
        q3=pi;
    else
    q3=acos(k3);
    end
    if mod(i,2)==1
        q(i,3)=q3;
    else
        q(i,3)=-q3;
    end
    %
    s2=((a3*cos(q(i,3))+a2)*n3-a3*sin(q(i,3))*m3)/(a2*a2+a3*a3+2*a2*a3*cos(q(i,3)));
    c2=(m3+a3*sin(q(i,3))*s2)/(a3*cos(q(i,3))+a2);
    q(i,2)=atan2(s2,c2);
    %
    s234=-sin(q(i,6))*(n(1)*cos(q(i,1))+n(2)*sin(q(i,1)))-cos(q(i,6))*(o(1)*cos(q(i,1))+o(2)*sin(q(i,1)));
    c234=o(3)*cos(q(i,6))+n(3)*sin(q(i,6));
    q(i,4)=atan2(s234,c234)-q(i,2)-q(i,3);
end
for i=1:8
    for j=1:6
        if q(i,j)<-pi
            q(i,j)=q(i,j)+2*pi;
        elseif q(i,j)>=pi
            q(i,j)=q(i,j)-2*pi;
        end
    end
end
q=q(~any(isnan(q),2),:);
q
```
