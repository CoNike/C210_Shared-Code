function T=circle_pose(p1,p2,p3,p0,t)
%此函数用于生成指向指定点轨迹的位姿连续矩阵
%p1,p2,p3为轨迹上三点，p0为指定的目标点，t为插值数目
%输出T为一系列连续的位姿
%% 输入值处理与判断
p1=p1(:);
p2=p2(:);
p3=p3(:);
p0=p0(:);
if length(p1)~=3||length(p2)~=3||length(p3)~=3||...
        length(p0)~=3
    error('输入数据格式不对')
end
%% 求解
[r0,~]=circle_r(p1,p2,p3);%求解圆形轨迹圆心
p=circle_jtraj(p1,p2,p3,t);%求解圆形轨迹的轨迹点
M=eye(4,4);%预设矩阵

for i=1:t
    z=p0-p(i,:)'; %指向目标点的方向
    p0r0=p0-r0;
    x=cross(p0r0,z);% p0p(i)r0的平面法向量
    %如果无法求解x
    if norm(x)<10^-10
        x=T{i-1}(1:3,4);
    end
    if i>1&&x'*x0<0%判断相邻x是否连续
        x=-x;
    end
    x0=x;
    y=cross(z,x);%生成y向量
    %p(i)点的位姿矩阵
    M(1:3,1)=x/norm(x);
    M(1:3,2)=y/norm(y);
    M(1:3,3)=z/norm(z);
    M(1:3,4)=p(i,:)';
    T{i}=M;
end
end