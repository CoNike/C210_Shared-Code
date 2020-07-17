%基于指数积模型的机器人全局可操作度（global manipulability index,GMI）求解
%gmi=GMI(robot,q)
%robot为机器人模型，为SerialLink类
%q为机器人关节角度序列或者角度空间轨迹序列

%参考文献为熊有伦等著的《机器人学》
%2020.5.1 黄洲洲
function gmi=GMI(robot,q,state,q0)
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
if strcmp(state,'q')
    %角度轨迹进行求解
    
    %判断输入角度是否符合标准
    if size(q,2)~=n
        error('q must have %d column.',n);
    end
    A=0;
    B=0;
    for i=1:size(q,1)-1
        %求解雅克比矩阵，雅克比矩阵的类型需要确认
        jaco_manu=robot.ManiIndex(q(i,:));
        A=A+jaco_manu*jaco_manu*delta_q(q(i,:),q(i+1,:));
        B=B+jaco_manu*delta_q(q(i,:),q(i+1,:));
    end
    gmi=A/B;
elseif strcmp(state,'T')
    if size(q,2)~=6
        error('输入格式有误')
    end
    for i=1:size(q,1)
        T=transl(q(i,1),q(i,2),q(i,3))*rpy2tr(q(i,4:6)*pi/180);
        qt(i,:)=robot.ikine_num_p(T,q0);
        q0=qt(i,:);
    end 
    A=0;
    B=0;
    for i=1:size(q,1)-1
        %求解雅克比矩阵，雅克比矩阵的类型需要确认
        jaco_manu=robot.ManiIndex(q(i,:));
        A=A+jaco_manu*jaco_manu*delta_q(q(i,:),q(i+1,:));
        B=B+jaco_manu*delta_q(q(i,:),q(i+1,:));
    end
    gmi=A/B;
else
    error('输入声明有误')
end

    function delta=delta_q(q1,q2)
        if length(q1)~=length(q2)
            error('输入角度的长度不一致')
        end
        iter=length(q1);
        delta=1;
        for ii=1:iter
            delta=delta*(q2(ii)-q1(ii));
        end
    end

end