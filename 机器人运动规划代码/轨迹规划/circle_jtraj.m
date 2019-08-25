function p=circle_jtraj(p1,p2,p3,t)
%���p1,p2,p3����֮���Բ���켣���켣��Ϊt��
[p0,r]=circle_r(p1,p2,p3);%���Բ�ĺͰ뾶
%��������غ�
if r==0
    p=ones(t,1)*p1(:)';
    return
end
%������Բ�ĵ�����
r1=p1-p0;
r2=p2-p0;
r3=p3-p0;
%���r3��r1���Լ�r2��r1֮��ļн�
theta1=atan2(cross(r1,r2)'*cross(r1,r2)/norm(cross(r1,r2)),r1'*r2);
theta2=atan2(cross(r1,r3)'*cross(r1,r2)/norm(cross(r1,r2)),r1'*r3);
%ȷ��theta2��theta1Ҫ��
if theta2<0
    theta2=theta2+2*pi;
end
if theta1>theta2
    theta1=2*pi-theta1;
    theta2=2*pi-theta2;
end

%�������ϵ���ڲ�ֵ
x1=r1/norm(r1)';
z1=cross(r1,r2)/norm(cross(r1,r2));
y1=cross(z1,x1);
%���Ƕ�����
ts=(0:t-1)/(t-1);
theta=ts*theta2;
p=zeros(t,3);%Ԥ���������
for i=1:t
    %sign(pi-theta1)��Ҫ����ȷ����ת����
    p(i,:)=(x1*cos(theta(i)*sign(pi-theta1))*r+y1*sin(theta(i)*sign(pi-theta1))*r)'+p0';
end
end