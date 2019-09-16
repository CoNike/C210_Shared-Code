function serialmupa=dh2poe(serialLink)
%���ڽ�DH����������ģ�ͣ�ת��Ϊ����ָ������ģ��
%% �ж�������Ƿ�ΪSerialLink
if ~isa(serialLink,'SerialLink')
    error('����������ǻ���DH�����Ļ�����')
end
%% ��ȡ��ز���
ro=serialLink;
%���Ը��ƵĲ���
name=ro.name;
% comment=ro.comment;
gravity=ro.gravity;
base=ro.base;
tool=ro.tool;
faces=ro.faces;
points=ro.points;
n=ro.n;
%�˶�ѧ����
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
    %��ȡ���˱任����
    %Ĭ�ϻ�������ϵ���һ���ؽ�����ϵ����һ��
    T=T*link.A(0);
end
T0=T;
%% ��ֵ
serialmupa=SerialManu(JL,'T0',T0,'name',name,'gravity',gravity,...
    'base',base,'tool',tool,'faces',faces,'points',points);
% serialmupa=SerialManu(JL,'T0',T0,'name',name,'comment',comment,'gravity',gravity,...
%     'base',base,'tool',tool,'faces',faces,'points',points);
end