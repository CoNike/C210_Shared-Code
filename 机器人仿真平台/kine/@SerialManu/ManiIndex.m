%基于指数积模型的机器人可操作度（manipulability）求解
%manipulability=ManiIndex(robot,q)
%robot为机器人模型，为SerialLink类
%q为机器人关节角度

%参考文献为熊有伦等著的《机器人学》
%2020.4.30 黄洲洲
function manipulability=ManiIndex(robot,q)
if ~isa(robot,'SerialManu')
    error('输入模型不对')
end
n=robot.n;
%判断输入角度是否符合标准
if size(q,2)~=n                         %此处使用size是为了以后扩展q从单一角度组到角度序列
    error('q must have %d column.',n);
end
%求解雅克比矩阵，雅克比矩阵的类型需要确认
jaco=robot.jacobp(q);

if n>=6  
    jaco_temp=jaco*jaco';
    manipulability=sqrt(abs(det(jaco_temp)));
else  %欠自由度机器人的可操作度需要文献确证
    jaco_temp=jaco'*jaco;
    manipulability=sqrt(abs(det(jaco_temp)));
end
end