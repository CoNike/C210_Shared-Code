function p=circle_jtraj(p1,p2,p3,t)
%求解p1,p2,p3三点之间的圆弧轨迹，轨迹点为t个
[p0,r]=circle_r(p1,p2,p3);%求解圆心和半径
%如果三点重合
if r==0
    p=ones(t,1)*p1(:)';
    return
end
%求解相对圆心的向量
r1=p1-p0;
r2=p2-p0;
r3=p3-p0;
%求解r3与r1，以及r2与r1之间的夹角
theta1=atan2(cross(r1,r2)'*cross(r1,r2)/norm(cross(r1,r2)),r1'*r2);
theta2=atan2(cross(r1,r3)'*cross(r1,r2)/norm(cross(r1,r2)),r1'*r3);
%确保theta2比theta1要大
if theta2<0
    theta2=theta2+2*pi;
end
if theta1>theta2
    theta1=2*pi-theta1;
    theta2=2*pi-theta2;
end

%求解坐标系便于插值
x1=r1/norm(r1)';
z1=cross(r1,r2)/norm(cross(r1,r2));
y1=cross(z1,x1);
%求解角度序列
ts=(0:t-1)/(t-1);
theta=ts*theta2;
p=zeros(t,3);%预设输出序列
for i=1:t
    %sign(pi-theta1)主要用于确定旋转方向
    p(i,:)=(x1*cos(theta(i)*sign(pi-theta1))*r+y1*sin(theta(i)*sign(pi-theta1))*r)'+p0';
end
end