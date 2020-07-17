%求解机器人基于指数积公式的正运动学
%T=fkinep(robot,q，var)
%其中robot为基于指数积建立的机器人模型（SerialLink）；
%q为关节角度；
%var，'rad'or 'deg'用于规定角度的单位制
%T为4X4矩阵

%指数积公式的参考文献：
%熊有伦等《机器人学：建模控制与视觉》
%李泽湘等《机器人学的几何基础》

%creator:Huang Zhouzhou   Time:2019/9/6
%Huazhong University of Science and Technology
function T=fkinep(robot,q,var)
%% 验证输入是否正确
%判断输入的模型是否正确
if ~isa(robot,'SerialManu')
    error('输入模型不对')
end
n=robot.n;
%判断输入角度是否符合标准
if size(q,2)~=n
    error('q must have %d column.',n);
end
%判断输入角度的单位制
angleunit='rad';
if nargin ==3
    angleunit=var;
end
if strcmp(angleunit,'deg')
    q=q*pi/180;
elseif ~strcmp(angleunit,'rad')
    error('角度声明有误')
end

%% 求解正运动学
T=robot.base;
for i=1:n
    link=robot.jolinks(i);
    T=T*link.isom(q(i));
end
T=T*robot.T0*robot.tool;
end