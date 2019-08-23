function Tor=idyna_nr_mdh(robot,q,qd,qdd,gravity,f_end)
%基于DH模型下牛顿欧拉迭代算法的机器人逆动力学求解。
%目前只支持数字解。
% Tor=idyna_nr_mdh(robot,q,qd,qdd,gravity,f_end)
%输入robot为机器人模型，包含有运动学参数和动力学参数；
%输入q为关节角度（1XN），qd为关节角加速度,qdd为对应关节角加速度；
%gravity为重力加速度，f_end为末端执行器施加载荷；
%这两个在不输入时会选用默认值；
%输出Torque为关节扭矩。
%函数参考了Peter,corke的RTB中的rne函数，
%参考书籍为机器人学：建模控制与视觉7.7节。
%% 设置和获取基本参数
n=robot.n; %机器人连杆数目
z0=[0,0,1]'; %基坐标系，z0的方向
grav= robot.gravity; %重力加速度，可以设置
fend = zeros(6, 1);%末端施加的载荷,W=[fx Fy Fz Mx My Mz]';
debug1=1;%设置调试参数
debug2=1;
if nargin>4%判断是否输入重力加速度
    grav=gravity;
end
if nargin==6%判断外部是否施加载荷
    fend=f_end;
end
%% 判断输入是否符合要求
if numcols(q)~=n||numcols(qd)~=n||numcols(qd)~=n||numrows(q)~=1||...
        numrows(qd)~=1||numrows(qdd)~= 1||length(fend)~=6
    error('输入数据有误')
end
%% 求解
%初始化部分数据
R={};%存储相邻旋转矩阵
Fm = [];%存储中间力数据，质心处
Nm = [];%存储中间力矩数据，质心处
r_rela= [];%存储相邻连杆之间距离
w = zeros(3,1);%初始角速度
wd = zeros(3,1);%初始角加速度
vd = grav(:);%初始加速度，重力加速度
for i=1
    link=robot.links(i);
    Ti=link.A(q(i));
    switch link.type %根据连杆类型来获取连杆长度d的值
        case 'R'
            d = link.d;
        case 'P'
            d = q(i);
    end
    alpha=link.alpha;%获取连杆扭角
    r_rela=[link.a; -d*sin(alpha); d*cos(alpha)];%坐标系i+1相对于坐标系i的位置
    %和标准DH有所不同，因为坐标定义不同,而且变换顺序不同
    if i==1
       Ti=robot.base*Ti; %考虑到基系到1系矩阵变换
       r_rela= t2r(robot.base) * pm;%考虑到有效运动只有连杆部分，虽然位置有变化
    end
    %存储相关变量，以便调用
    r_temp(:,i)=r_rela;
    R{i}=t2r(Ti);
end
%为什么要存储相关变量，而不是将此处上下两个循环一起做？
%因为在求解下面一个循环中的某些参数时，会调用前者循环中i+1对应的数据？
for i=1:n
    link=robot.links(i);%获取连杆结构体
    Ri=R{i}';%此处需要转置，转置！！！但是在书中是以i相对于i+1的变换表示出来的，不是很明确
    r_rela=r_temp(:,i);%相邻连杆之间的位置
    r=link.r;%连杆质心相对于连杆的位置
    switch link.type
        case 'R'
            w=Ri*w+qd(i)*z0;%求解角速度%此处书中并没有用z0表示，而是i+1zi+1
            wd=Ri*wd+cross(Ri*w,qd(i)*z0)+qdd(i)*z0;%求解角加速度，但是书中公式存在歧义
            %第二项最好写为（）X（）的形式
            vd=Ri*(vd+cross(wd,r_rela)+cross(w,cross(w,r_rela)));%加速度求解
        case 'P'
            w=Ri*w;
            wd=Ri*wd;
            vd=Ri*(vd+cross(wd,r_rela)+cross(w,cross(w,r_rela)))+...
                2*cross(w,qd(i)*z0)+qdd(i)*z0;
    end
    vd_cen=cross(wd,r)+cross(w,cross(w,r))+vd;%连杆质心处的加速度
    F=link.m*vd_cen;%连杆质心处受力
    N=link.I*wd+cross(w,link.I*w);%连杆质心处扭矩
    %存储相关数据以便调用
    Fm=[Fm F];
    Nm=[Nm N];
end
%调试输出，不用时可以注释掉
if debug1==1
    fprintf('R:');disp(R)
    fprintf('Fm:');disp(Fm)
    fprintf('Nm:');disp(Nm)
    fprintf('\n');
end
%求解关节力矩
%获取关节力和力矩
fend=fend(:);
ff=fend(1:3);
fn=fend(4:6);
for i=n:-1:1
    link=robot.links(i);%获取连杆结构体
    r=link.r;%获取连杆质心位置
    %对于这两个变量的获取如书中公式是要比其他变量的循环数+1
    %相邻连杆是因为改进DH的定义不同？
    if i==n
        Ri=eye(3);
        r_rela=[ 0 0 0]';
    else
        Ri=R{i+1};%获取旋转矩阵，不需要转置
        r_rela=r_temp(:,i+1);
    end
    %注意ff与fn的求解顺序不能反了
    fn=Ri*fn+Nm(:,i)+cross(r,Fm(:,i))+cross(r_rela,Ri*ff);%关节力矩
    ff=Ri*ff+Fm(:,i);%关节力
    if debug2==1
        fprintf('fn:');disp(fn)
        fprintf('ff:');disp(ff)
        fprintf('\n');
    end
    switch link.type
        %存在和标准DH一样的问题
        case 'R'
            Tor(i)=fn'*z0-link.friction(qd(i))+link.G^2 * link.Jm*qdd(i);
        case 'P'
            Tor(i)=ff'*z0-link.friction(qd(i))+link.G^2 * link.Jm*qdd(i);
    end
end
end