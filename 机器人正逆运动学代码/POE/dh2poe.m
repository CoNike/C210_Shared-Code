function serialmupa=dh2poe(serialLink)
%用于将DH参数建立的模型，转换为基于指数积的模型
%% 判断输入的是否为SerialLink
if ~isa(serialLink,'SerialLink')
    error('输入变量不是基于DH建立的机器人')
end
%% 获取相关参数
ro=serialLink;
%可以复制的参数
name=ro.name;
% comment=ro.comment;
gravity=ro.gravity;
base=ro.base;
tool=ro.tool;
faces=ro.faces;
points=ro.points;
n=ro.n;
%运动学参数
T=SE3(eye(4));
for i=1:n
    link=ro.links(i);
    offset=link.offset;
    lname=link.name;
    flip=link.flip;
    qlim=link.qlim;
    I=link.I;
    m=link.m;
    rc=link.r;
    Jm=link.Jm;
    B=link.B;
    Tc=link.Tc;
    G=link.G;
    if strcmp(link.jointtype,'R')
        w=T.a;
        r=T.t;
        jolink=JoLink('w',w,'r',r,'offset',offset,'name',lname,'flip',flip...
            ,'qlim',qlim,'m',m,'rc',rc,'Jm',Jm,'B',B,'Tc',Tc,'G',G,'I',I);
    else
        v=T.a;
        jolink=JoLink('v',v,'offset',offset,'name',lname,'flip',flip...
            ,'qlim',qlim,'m',m,'rc',rc,'Jm',Jm,'B',B,'Tc',Tc,'G',G,'I',I);
    end
    JL(i)=jolink;
    %求取连杆变换矩阵
    %默认基础坐标系与第一个关节坐标系方向一致
    T=T*link.A(0);
end
T0=T;
%% 赋值
serialmupa=SerialManu(JL,'T0',T0,'name',name,'gravity',gravity,...
    'base',base,'tool',tool,'faces',faces,'points',points);
% serialmupa=SerialManu(JL,'T0',T0,'name',name,'comment',comment,'gravity',gravity,...
%     'base',base,'tool',tool,'faces',faces,'points',points);
end