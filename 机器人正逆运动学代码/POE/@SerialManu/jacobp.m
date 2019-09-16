%基于指数积的机器人雅克比矩阵的求解
%J=jacobp(robot,q,var)
%robot为机器人模型，为SerialLink类
%q为机器人关节角度
%var 用于声明机器人雅克比矩阵的类型包括：
%s,基坐标系空间雅克比矩阵求解
%b，物体坐标系下的雅克比矩阵
%d，基坐标系下的雅克比矩阵




function J=jacobp(robot,q,var)
%% 输入判断与选择
if isa(robot,'SerialLink')
    error('输入模型不对')
end
n=robot.n;
%判断输入角度是否符合标准
if size(q,2)~=n
    error('q must have %d column.',n);
end
%% 雅克比求解
T=eye(4);
tw=robot.twist;
for i=1:n
    link=robot.jolinks(i);
    ad=adjoint(T);
    J(:,i)=ad*tw(i,:)';
    T=T*link.isom(q(i));
end
%% 判断输出
if nargin==2
    argu='s';
else
    argu=var;
end
if strcmp(argu,'s')%空间雅可比矩阵
    return
elseif strcmp(argu,'b') %求解物体雅可比矩阵
    Tq=T*robot.T0;
    J=adjoint(inv(Tq))*J;
elseif  strcmp(argu,'d') 
    Tq=T*robot.T0;
    J=adjoint(inv(Tq))*J;
    R=Tq(1:3,1:3);
    J=[R zeros(3,3);
        zeros(3,3) R]*J;
else
    error('声明类型有误')
end
end

function T6=adjoint(T4)
if size(T4)~=[4,4]
    error('输入矩阵不是4X4');
end
r=T4(1:3,1:3);
p=T4(1:3,4);
T6=[r skew0(p)*r;
    zeros(3,3) r];
end