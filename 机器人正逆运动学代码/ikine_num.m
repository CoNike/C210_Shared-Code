% ikine_num
function [q,k,dis]=ikine_num(robot,Tg,q0)
%求解机器人的数值解,只能依据迭代过程求出一组解；
%因此无法获得所有解，有时候也无法获得自己想要的解。
%作为测试版本，该程序中包含了冗余的参数。
%输入为robot为机器人结构体，包含机器人的运动学参数；
%Tg为用齐次矩阵表达的目标位姿，为4X4double或者SE3
%q0为迭代的起始点，如果不输入，则为[0 0 0 0 0 0];
n=robot.n; %获取机器人连杆数
%设定初始角度
qi=zeros(1,6);
if nargin==3
    qi=q0;
end
if length(qi)~=n
    error('输入角度有误')
end
%转换Tg为4X4double，便于移植到其他语言
if isa(Tg,'SE3')
    Tg=Tg.T;
end
k=1;
flag=true;%设置循环标志
dis2=1;
while flag
    T0=robot.fkine(qi);%求解初始状态时位姿矩阵
    T0=T0.T;
    T=Tg*inv(T0);
    v=T(1:3,4);         %求出变刚体变换对应的平移量
    w=vex(t2r(T)-eye(3)); %将机器人旋转矩阵转换为轴角的乘积
    delta=[v;w];        %合并为6维矢量
    dis1=norm(delta);    %求delta模长作为判断值
    jaco=robot.jacob0(qi);  %求解机器人q0时的雅克比矩阵
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
        beta=0.3;
    else
        beta=0.92;
    end
    qi=qi+alpha*temp'*beta;          %求解更新后的角度
    for i=length(qi)
        if qi(i)>=pi
            qi(i)=qi(i)-2*pi;
        elseif qi(i)<-pi
            qi(i)=qi(i)+2*pi;
        end
            
    end
    if ((abs(dis2-dis1))<10^-6)||(k>1000)        %判断是否结束迭代
        flag=false;
    end
    dis2=dis1;
    if nargout>1
    dis(k)=dis2;
    k=k+1;
    end
end
q=qi;
% plot(dis)
end