%基于指数积模型的机器人条件数（condition number）求解
%connum=ConNumIndex(robot,q)
%robot为机器人模型，为SerialLink类
%q为机器人关节角度

%参考文献为熊有伦等著的《机器人学》
%2020.5.1黄洲洲
function connum=ConNumIndex(robot,q)
if ~isa(robot,'SerialManu')
    error('输入模型不对')
end
n=robot.n;
%判断输入角度是否符合标准
if size(q,2)~=n                         %此处使用size是为了以后扩展q从单一角度组到角度序列
    error('q must have %d column.',n);
end
%求解雅克比矩阵，雅克比矩阵的类型需要确认
for i=1:size(q,1)
    jaco=robot.jacobp(q(i,:));
    [~,V,~]=svd(jaco);
    connum(i)=V(n,n)/V(1,1);
end
end