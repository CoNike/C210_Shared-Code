function Tor=idyna_nr_dh(robot,q,qd,qdd,gravity,f_end)
%基于DH模型下牛顿欧拉迭代算法的机器人逆动力学求解
%目前只支持数字解
%Tor=idyna_nr_dh(robot,q,qd,qdd,gravity,f_end)
%输入robot为机器人模型，包含有运动学参数和动力学参数
%输入q为关节角度（1XN），qd为关节角加速度,qdd为对应关节角加速度
%gravity为重力加速度，f_end为末端执行器施加载荷，
%这两个在不输入时会选用默认值
%输出Torque为关节扭矩
%函数参考了Peter,corke的RTB中的rne函数
%参考书籍为机器人学：建模规划与控制7.6节
%% 设置和获取基本参数
%确保输入角度，角速度，角加速度为行向量
q=q(:)';
qd=qd(:)';
qdd=qdd(:)';
n=robot.n; %机器人连杆数目
z0=[0,0,1]'; %基坐标系，z0的方向
grav= robot.gravity; %重力加速度，可以设置
fend = zeros(6, 1);%末端施加的载荷,W=[fx Fy Fz Mx My Mz]';
debug1=0;%设置调试参数
debug2=0;
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
%求解相关矩阵和参数
R={};

for i=1:n
    link=robot.links(i);%获取连杆（结构体）
    Ti=link.A(q(i));%获取相邻连杆之间的变换矩阵
    switch link.type %根据连杆类型来获取连杆长度d的值
        case 'R'
            d = link.d;
        case 'P'
            d = q(i);
    end
    alpha=link.alpha;%获取连杆扭角
    r_rela=[link.a; d*sin(alpha); d*cos(alpha)];%获取连杆坐标系i+1相对于连杆坐标系i
    %的相对位置在i+1系中的表示。是否可以获取位置然后直接相减再求变换（R0,i'*(Oi-Oi-1)）
    r_temp(i,:)=r_rela;%暂存相对位置数据方便后续调用
    R{i}=Ti.t2r;%存储相邻连杆之间的旋转变换矩阵方便调用
end
%输出值的初始化
Tor=zeros(1,n);
%存储扭矩和加速度
Fm = [];
Nm = [];
%部分值的初始化
Rb = t2r(robot.base)';%获取连杆的base的变换矩阵的旋转部分
w = Rb*zeros(3,1);%初始化连杆基座角速度 %为啥不直接用zeros(3,1)初始化
wd = Rb*zeros(3,1);%初始化连杆基座的角加速度%也是同样的问题
vd = Rb*grav(:);%初始化基座的加速度
%能否用矩阵对相关数据进行储存


%正向递推加求速度，角速度，角加速度
for i=1:n
    link=robot.links(i);%获取连杆的结构体
    r_rela=r_temp(i,:)';%获取相邻连杆之间的向量
    Ri=R{i}';%获取相邻的变换矩阵,记得转置因为要获取的是逆矩阵
    r=link.r';%质心C相对于连杆坐标系的位置向量,注意为行向量还是列向量
    switch link.type
        case 'R' %旋转关节
            %注意角速度角加速度的顺序，因为角加速度会调用之前的角速度
            wd=Ri*(wd+qdd(i)*z0+qd(i)*cross(w,z0));%求解角加速度 
            w=Ri*(w+qd(i)*z0); %求解角速度
            vd=Ri*vd+cross(wd,r_rela)+cross(w,cross(w,r_rela)); %求解加速度
        case 'P'
            wd=Ri*wd;
            w=Ri*w;
            vd=Ri*(vd+qdd(i)*z0)+2*qd(i)*cross(w,Ri*z0)...
                +cross(wd,r_rela)+cross(w,cross(w,r_rela));
    end
    vd_cen=vd+cross(wd,r)+cross(w,cross(w,r));%求解质心的加速度
    F=link.m*vd_cen;%由加速度产生的力
    N=link.I*wd+cross(w,link.I*w);%由旋转产生的连杆力矩
    %存储相关数据
    Fm=[Fm F];
    Nm=[Nm N];
end
if debug1==1
    fprintf('Fm:');disp(Fm)
    fprintf('Nm:');disp(Nm)
    fprintf('\n');
end
%逆推求解关节力
%获取末端力和力矩
f_end=fend(:);
ff=f_end(1:3);%末端力
fn=f_end(4:6);%末端力矩
for i=n:-1:1
    link=robot.links(i);%获取连杆结构体
    r_rela=r_temp(i,:)';%获取相邻连杆之间的向量
    r=link.r';%获取质心相对于连杆的参数
    %获取相邻连杆变换矩阵
    if i==n
        Ri=eye(3);%i=n时,对应的连杆矩阵为Rn,n,其为单位阵
    else
        Ri=R{i+1};%i<n时,对应的连杆矩阵为Rn,n+1
    end
    %注意求解顺序
    %fn=cross(r_rela+r,Fm(:,i))+Ri*(fn+cross(Ri'*r_rela,ff))+Nm(:,i);%计算关节输出力矩
    fn=cross(r_rela+r,Fm(:,i))+Ri*(fn)+cross(r_rela,Ri*ff)+Nm(:,i);%计算关节输出力矩
    ff=Ri*ff+Fm(:,i);%计算关节输出力
    if debug2==1
    fprintf('ff:');disp(ff)
    fprintf('fn:');disp(fn)
    fprintf('\n');
    end
    Ri=R{i};
    switch link.type
        %为什么需要乘R？？因为关节i和坐标系i-1对应
        case 'R'
            
            %t=fn'*(Ri'*z0); %验证拉格朗日
             t=fn'*(Ri'*z0)-link.friction(qd(i))+link.G^2 * link.Jm*qdd(i);%关节力矩
            %并没有考虑到电机自身转速与连杆转速的相互影响,且只考虑粘滞摩擦，后续可以添加
        case 'P'
            t=ff*(Ri'*z0)-link.friction(qd(i))+link.G^2 * link.Jm*qdd(i);
            %结合旋转关节求解，可知此处求解时电机转速的耦合是需要考虑的
    end
    Tor(i)=t;%输出力矩
end
    %% 求解基座力
    R1 = R{1};
    fn = R1*(fn);
    ff= R1*ff;
    wbase = [ff; fn];
end
