function T=tracj_l(T0,Tg,t,argument)
%在两个位姿之间生成均匀的直线插值
%输入T0和Tg都是4*4的刚体变换矩阵，t为时间序列或者步数
%输出T为连续的rpy角和位置序列（Nx6）
%argumrnt用于声明旋转矩阵的表示类型，其中包括
%      'rpy'用rpy角表征旋转矩阵；
%      'angvec'用旋转轴和角度的形式表征旋转矩阵
%      'eul'用欧拉角表示旋转矩阵
%       若不输入，则默认按rpy

%% 标准化t，判断t是否符合标准
%将两种不同类型的t都转换为0~1的数组
if length(t)==1
    ts=(0:t-1)/(t-1);
else
    tmax=max(t);
    tmin=min(t);
    if tmin<=0
        error('输入时间序列不符合要求')
    end 
    ts=(t-tmin)/(tmax-tmin);
end
%判断是否输入角度声明，若没有则按rpy处理
if nargin==3
    argument='rpy';
elseif ~ischar(argument)
    error('变量四请输入一个类型')
end
%% 数据转换
%获取起始旋转矩阵和平移向量
R0=t2r(T0);
Rg=t2r(Tg);

t0=transl(T0);
tg=transl(Tg);
%求解相对变换矩阵
R=Rg*inv(R0);
%预设输出变量大小
rpy=zeros(length(t),3);
if strcmp(argument,'rpy')  %判断char是否符合的函数
    rpy0=tr2rpy(R); %将旋转矩阵转换为rpy
    for i=1:length(ts)
        rpy_temp=rpy0*ts(i); %求中间序列的rpy
        R_temp=rpy2r(rpy_temp)*R0;%求中间序列的旋转矩阵
        rpy_temp2=tr2rpy(R_temp);%求解对应的rpy
        rpy(i,:)=rpy_temp2(:)';
    end
elseif strcmp(argument,'angvec')
    [theta,vector]=tr2angvec(R);%将矩阵转换为轴角形式
    for i=1:length(ts)
        theta_temp=theta*ts(i);%求解中间角度
        R_temp=angvec2r(theta_temp,vector)*R0; %求解中间矩阵
        rpy_temp2=tr2rpy(R_temp);%转换为rpy
        rpy(i,:)=rpy_temp2(:)';
    end
elseif  strcmp(argument,'eul')
    eul0=tr2eul(R);%将矩阵转换为欧拉角
    for i=1:length(ts)
        rpy_temp=eul0*ts(i);%求解中间欧拉角
        R_temp=rpy2r(rpy_temp)*R0;%转换为中间矩阵
        rpy_temp2=tr2rpy(R_temp);%转换为rpy
        rpy(i,:)=rpy_temp2(:)';
    end
else     %判断是否输入错误类型
    error('请输入合适的角度表达类型')
end

%% 生成输出序列
t=ts'*(tg-t0)'+t0';
T=[rpy t];
end