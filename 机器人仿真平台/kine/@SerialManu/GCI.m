%基于指数积模型的机器人全局条件数（global manipulability index,GMI）求解
%gmi=GMI(robot,q)
%robot为机器人模型，为SerialLink类
%q为机器人关节角度序列或者角度空间轨迹序列

%参考文献为熊有伦等著的《机器人学》
%2020.5.1 黄洲洲
function gci=GCI(robot,q,state,q0)
if ~isa(robot,'SerialManu')
    error('输入模型不对')
end
if nargin==2
    state='q';
elseif nargin==3 %若为末端轨迹，且不输入参考角度用于确定逆运动学，则默认为机器人初始角度
    q0=robot.offset;
elseif nargin==4
    if strcmp(state,'T')
        error('输入类型与目标不匹配')
    end
end
n=robot.n;
%% 求解
gci=0;
if strcmp(state,'q')
    %角度轨迹进行求解
    
    %判断输入角度是否符合标准
    if size(q,2)~=n
        error('q must have %d column.',n);
    end
    
    for i=1:size(q,1)
        gci=gci+robot.ConNumIndex(q(i,:));
    end
    gci=gci/size(q,1);
elseif strcmp(state,'T')
    if size(q,2)~=6
        error('输入格式有误')
    end
    for i=1:size(q,1)
        T=transl(q(i,1),q(i,2),q(i,3))*rpy2tr(q(i,4:6)*pi/180);
        qtemp=robot.ikine_num_p(T,q0);
        gci=gci+robot.ConNumIdex(qtemp);
        q0=qtemp;
    end
    gci=gci/size(q,1);
else
    error('输入声明有误')
end

end