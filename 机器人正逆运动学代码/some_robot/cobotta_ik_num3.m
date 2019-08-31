% cobotta
function [q,n,dis]=cobotta_ik_num3(TG,q0,beta)

%判断是否设置迭代起点，若没有设置，则默认为[0 0 0 0 0 0]
if nargin==1
    q0=[0 0 0 0 0 0];
end

%设置cobotta相关参数
d1=0.18;
a2=0.165;
a3=-0.012;
d3=0.02;
d4=0.1775;
d5=-0.0645;
d6=0.045;
L1= Link('revolute', 'd', d1,     'a', 0,        'alpha', -pi/2,'offset',0);
L2= Link('revolute', 'd', 0,         'a', a2,    'alpha', 0,'offset',-pi/2);
L3= Link('revolute', 'd', d3,         'a', a3,   'alpha', pi/2,'offset',pi/2);
L4= Link('revolute', 'd', d4,    'a', 0,        'alpha', -pi/2,'offset',0);
L5= Link('revolute', 'd', d5,    'a', 0,        'alpha', pi/2,'offset',0);
L6= Link('revolute', 'd', d6,     'a', 0,        'alpha', 0,'offset',0);
cobotta=SerialLink([L1,L2,L3,L4,L5,L6]);

TG=SE3(TG);
%设置迭代相关对的参数
dis1=0;
dis2=0;%目标点与迭代点距离向量的初始设置
n=2;
flag=1;%用于判断是否循环的标志
% beta=1;
lambda=0.01;%设置避免奇异位置的参数
while flag==1
    T0=cobotta.fkine(q0); %迭代初始角度对应的空间位置
    T=TG*inv(T0); %求初始位置到目标位置的变换
    v=T.t;         %求出变刚体变换对应的平移量
    w=vex(T.t2r-eye(3)); %将机器人旋转矩阵转换为轴角的乘积
    delta=[v;w];        %合并为6维矢量
    dis1=norm(delta);    %求delta模长作为判断值
    jaco=cobotta.jacob0(q0);  %求解机器人q0时的雅克比矩阵
    det_j=abs(det(jaco));   %求解雅克比行列式的绝对值
    %根据det_j采取不同的迭代方法
    if  det_j>10^-4
        temp=inv(jaco)*delta;
        alpha=1;
    else
        %奇异位置附近采用的方法
        lambda=0.01;
        temp=jaco'*inv(jaco*jaco'+lambda*eye(6))*delta;
        alpha=1.5;
    end
    if dis1>0.1
        beta=beta;
    else
        beta=0.92;
    end
    q0=q0+alpha*temp'*beta;          %求解更新后的角度
    for i=length(q0)
        if q0(i)>=pi
            q0(i)=q0(i)-2*pi;
        elseif q0(i)<-pi
            q0(i)=q0(i)+2*pi;
        end
            
    end
    if ((abs(dis2-dis1))<10^-6)||(n>1000)        %判断是否结束迭代
        flag=0;
    end
    dis2=dis1;
    dis(n)=dis2;
    n=n+1;
end
q=q0;
% plot(dis)
end