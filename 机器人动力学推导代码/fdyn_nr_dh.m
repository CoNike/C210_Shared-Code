function qdd=fdyn_nr_dh(robot,q,qd,torque,gravity,f_end)
%DH模型下，基于牛顿欧拉迭代方法的机器人正运动学求解
%目前只支持数字求解
%输入robot为机器人模型，包含有运动学参数和动力学参数
%输入q为关节角度（1XN），qd为关节角加速度,torque为关节扭矩
%gravity为重力加速度，f_end为末端执行器施加载荷，
%这两个在不输入时会选用默认值
%输出qdd为对应关节角加速度
%函数参考了Peter,corke的RTB中的accel函数
%参考书籍为机器人学：建模规划与控制7.6节
%% 设置和获取基本参数
n=robot.n; %机器人连杆数目
grav= robot.gravity; %重力加速度，可以设置
fend = zeros(6, 1);%末端施加的载荷,W=[fx Fy Fz Mx My Mz]';
debug1=0;%设置调试参数
if nargin>4%判断是否输入重力加速度
    grav=gravity;
end
if nargin==6%判断外部是否施加载荷
    fend=f_end;
end
%% 判断输入是否符合要求
if numcols(q)~=n||numcols(qd)~=n||numcols(qd)~=n||numrows(q)~=1||...
        numrows(qd)~=1||numrows(torque)~= 1||length(fend)~=6
    error('输入数据有误')
end
%% 求解
%预设B（q）求解所需要的参数
qd_temp=zeros(1,n);
qdd_temp=eye(n);
%求解B（q）
%求解需要考虑机器人DH模型是否采用了改进版本，求torque的相对值也是
for i=1:n
    if robot.mdh == 0
        M(i,:)=idyna_nr_dh(robot,q,qd_temp,qdd_temp(:,i),[0 0 0],fend); %此处是否应该包含fend可能存疑
    else
        M(i,:)=idyna_nr_mdh(robot,q,qd_temp,qdd_temp(:,i),[0 0 0],fend);%此处是否应该包含fend可能存疑
    end
end
%求解加速度
if robot.mdh == 0
    tor_temp = idyna_nr_dh(robot, q, qd, zeros(1,n),grav,fend);%求torque的相对值
else
    tor_temp = idyna_nr_mdh(robot, q, qd, zeros(1,n),grav,fend);%求torque的相对值
end
if debug1==1
    fprintf('M:');disp(M)
    fprintf('tor_temp:');disp(tor_temp)
    fprintf('\n');
end
qdd=M\(torque-tor_temp)';%求解加速度
end