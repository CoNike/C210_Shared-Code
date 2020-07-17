%基于指数积的机器人逆运动学数值求解
%q=ikine_num_p(robot,Tg,q0)


function q=ikine_num_p(robot,Tg,q0)
%% 验证输入是否正确
%判断输入的模型是否正确
if ~isa(robot,'SerialManu')
    error('输入模型不对')
end
n=robot.n;
%判断输入角度是否符合标准
if size(q0,2)~=n
    error('q must have %d column.',n);
end
%判断Tg是否符合要求
if isa(Tg,'SE3')
    Tg=double(Tg);
elseif(size(Tg)~=[4,4])
    error('请输入位姿矩阵')
end

flag1=0;%显示末端距离
%% 求解
flag=true;
dis1=10000;
k=0;
while flag
    k=k+1;% 确定迭代次数
    T0=robot.fkinep(q0);%q0时末端位姿
    
    %求解相对位置的六维矢量
    T=Tg/T0;
    v=T(1:3,4);
    R=T(1:3,1:3);
    w=iskew(R);
    delta=[v;w];
    jaco=robot.jacobp(q0,'d');%求解雅克比
    %考虑非六轴机器人
    if n>6
        jaco_det=jaco*jaco';
    else
        jaco_det=jaco'*jaco;
    end
    det_j=sqrt(abs(det(jaco_det)));   %求解雅克比行列式的绝对值
    
    %考虑非六轴机器人的优化
    if n>6
        %根据det_j采取不同的迭代方法
        if  det_j>10^-4
            temp=jaco'*inv(jaco*jaco')*delta;
            alpha=1;
        else
            %奇异位置附近采用的方法
            lambda=0.01;
            temp=jaco'*inv(jaco*jaco'+lambda*eye(6))*delta;
            alpha=1.5;
        end
    elseif n<6
        if  det_j>10^-4
            temp=inv(jaco'*jaco)*jaco'*delta;
            alpha=1;
        else
            %奇异位置附近采用的方法
            lambda=0.01;
            temp=inv(jaco'*jaco+lambda*eye(n))*jaco'*delta;
            alpha=1.5;
        end
    else
        if  det_j>10^-4
            temp=inv(jaco)*delta;
            alpha=1;
        else
            %奇异位置附近采用的方法
            lambda=0.01;
            temp=jaco'*inv(jaco*jaco'+lambda*eye(6))*delta;
            alpha=1.5;
        end
    end
    %根据相对位置选择迭代步长
     if dis1>0.1
        beta=0.3;
    else
        beta=0.92;
    end
    q0=q0+alpha*temp'*beta;          %求解更新后的角度
    %避免角度超出限制
    qlim=robot.qlim;
    for i=1:length(q0)
        if strcmp(robot.jolinks(i).jointtype,'R')
            if q0(i)>=qlim(i,2)*pi/180
                q0(i)=q0(i)-2*pi;
            elseif q0(i)<-pi
                q0(i)=q0(i)+2*pi;
            end
        end
    end
    %记录距离
    dis2=dis1;
    dis1=norm(delta);
    %判断是否结束迭代
    if (((abs(dis2-dis1))<10^-6)&& (dis1<10^-6))||(k>1000)        
        flag=false;
    end
    if flag1==1
        dis1
        q0
    end
end
q=q0;

    function w=iskew(R)
        w=[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)]/2;
    end
end