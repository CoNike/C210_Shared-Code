function q=ANCHUAN_ikine(TG)


d1=0.33;
a1=0.088;
a2=0.31;
a3=0.04;
d4=-0.305;
d6=-0.08;


% L1 = Link('d', 0.33, 'a', 0.088, 'alpha', -pi/2);      %offset',pi/2);
% L2 = Link('d', 0, 'a', 0.31, 'alpha', pi,'offset',-pi/2);     %offset',pi/4);
% L3 = Link('d', 0, 'a', 0.040, 'alpha', -pi/2);
% L4 = Link('d', -0.305, 'a', 0, 'alpha', pi/2);
% L5 = Link('d', 0, 'a', 0, 'alpha', -pi/2);
% L6 = Link('d', -0.08, 'a', 0, 'alpha', pi);
% robot=SerialLink([L1,L2,L3,L4,L5,L6]);
% robot.name = 'Motoman robot';
% q0=[0 0 0 0 -pi/2 0];
% TG=robot.fkine(q0);
TG=SE3(TG);
n=TG.n;
o=TG.o;
a=TG.a;
p=TG.t;

p=p-a*abs(d6);
q1_1=atan2(p(2),p(1));
q1_2=atan2(-p(2),-p(1));
q=zeros(16,6);
q(1:8,1)=q1_1;
q(9:16,1)=q1_2;
for i=1:2
    m1=p(1)-a1*cos(q(8*i,1));
    n1=p(2)-a1*sin(q(8*i,1));
    p1=p(3)-d1;
    m3=2*a2*a3;
    n3=2*a2*d4;
    p3=m1*m1+n1*n1+p1*p1-a3*a3-d4*d4-a2*a2; 
    if (m3*m3+n3*n3-p3*p3)<0
        q(8*i-7:8*i,3)=nan;
    else
    q3_1=atan2(m3,n3)-atan2(p3,sqrt(m3*m3+n3*n3-p3*p3));
    q(8*i-7:8*i-3,3)=q3_1;
    q3_2=atan2(m3,n3)-atan2(p3,-sqrt(m3*m3+n3*n3-p3*p3));
    q(8*i-3:8*i,3)=q3_2;
    end
end


for i=1:8
    m2=a2+a3*cos(q(2*i,3))-d4*sin(q(2*i,3));
    n2=-d4*cos(q(2*i,3))-a3*sin(q(2*i,3));
    
     if mod(i,2)==1
          q2=atan2(m2,n2)-atan2(p1,sqrt(m2^2+n2^2-p1^2)); 
          q(2*i-1:2*i,2)=q2;
     else
           q2=atan2(m2,n2)-atan2(p1,-sqrt(m2^2+n2^2-p1^2)); 
          q(2*i-1:2*i,2)=q2;
     end
end
% q=q(~any(isnan(q),2),:);
[m,~]=size(q);
for i=1:m
k=a1+a2*sin(q(i,2))+a2*sin(q(i,2)-q(i,3))-d4*cos(q(i,2)-q(i,3));
q1_1=atan2(p(2)/k,p(1)/k);
if abs(q(i,1)-q1_1)>10^-6
    q(i,1)=nan;
end
end
q=q(~any(isnan(q),2),:);
[m,~]=size(q);
for i=1:m
c5=a(3)*sin(q(i,2)-q(i,3))-a(1)*cos(q(i,2)-q(i,3))*cos(q(i,1))-a(2)*cos(q(i,2)-q(i,3))*sin(q(i,1));
     if mod(i,2)==1
          q5=acos(-c5);
          q(i,5)=q5;
     else
          q5=-acos(-c5);
          q(i,5)=q5;
     end
end


for i=1:m
    c6=(n(3)*sin(q(i,2)-q(i,3))-n(1)*cos(q(i,2)-q(i,3))*cos(q(i,1))-n(2)*cos(q(i,2)-q(i,3))*sin(q(i,1)))/sin(q(i,5));
    s6=(o(3)*sin(q(i,2)-q(i,3))-o(1)*cos(q(i,2)-q(i,3))*cos(q(i,1))-o(2)*cos(q(i,2)-q(i,3))*sin(q(i,1)))/sin(q(i,5));
    q6=atan2(s6,c6);
    q(i,6)=q6;
    s4=(a(2)*cos(q(i,1))-a(1)*sin(q(i,1)))/sin(q(i,5));
    c4=(a(3)*cos(q(i,2)-q(i,3))+a(1)*sin(q(i,2)-q(i,3))*cos(q(i,1))+a(2)*sin(q(i,2)-q(i,3))*sin(q(i,1)))/sin(q(i,5));
    q4=atan2(s4,c4);
    q(i,4)=q4;
end

[k1,j1]=size(q);
for k=1:k1
    for j=1:j1
        if q(k,j)<-pi
            q(k,j)= q(k,j)+2*pi;
        elseif q(k,j)>pi
            q(k,j)= q(k,j)-2*pi;
        end
    end
end



end
