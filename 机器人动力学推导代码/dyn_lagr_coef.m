function [D,H,G,fv]=dyn_lagr_coef(robot,q,qd,gravity)
%此函数用拉格朗日动力学方法求解机器人动力学,
%目前只考虑旋转关节,不考虑电机的影响。
%[D,H,G,fv,fc]=dyn_lagr_coef(robot,q,qd,qdd,gravity,f_end)
%输入robot为机器人模型，
%q为机器人关节变量，
%qd为机器人关节角速度，
%gravity为机器人重力向量
%f_end为机器人末端执行器的受力
%输出D为加速度项的系数矩阵，表征的是加速度对关节力的影响；
%输出H为速度项的系数矩阵，表征的是速度对各关节力的影响；
%输出G为重力项
%fv和fc为选择输出项
%输出fv为各关节的粘滞摩擦力
%输出fc为各关节的库伦摩擦力
%参考文献为熊有伦等著的《机器人学：建模、控制与视觉》和
%布鲁诺·西西里安诺等著的《机器人学：建模规划与控制》相关部分
%程序参考：ttps://www.jianshu.com/p/6d04539f1cfe
n=robot.n;  %关节数
if length(q)~=n && length(qd)~=qd
    error('输入数据维数不对')
end
gc=robot.gravity;%重力
if nargin==4
    gc=gravity;
end
%获取DH参数，并生成DH参数表
for i=1:n
    eval(['syms ','at',num2str(i),' real;']);
    eval(['syms ','d',num2str(i),' real;']);
    a(i)=eval(['at',num2str(i)]);
    d(i)=eval(['d',num2str(i)]);
end
alpha=robot.alpha;
offset=robot.offset;
dhtable=[alpha' a' d' offset'];
%生成符号矩阵
[D,H,G]=dyn_lagr_sym(dhtable);
%对符号进行赋值
for i=1:n
  link=robot.links(i);
  eval(['at',num2str(i),'=','link.a',';']);
  eval(['d',num2str(i),'=','link.d',';']);
  eval(['q',num2str(i),'=','q(i)',';']);
  eval(['dq',num2str(i),'=','qd(i)',';']);
  eval(['m',num2str(i),'=','link.m',';']);
  eval(['xc',num2str(i),'=','link.r(1)',';']);
  eval(['yc',num2str(i),'=','link.r(2)',';']);
  eval(['zc',num2str(i),'=','link.r(3)',';']);
  eval(['Ix',num2str(i),'=','link.I(1,1)',';']);
  eval(['Iy',num2str(i),'=','link.I(2,2)',';']);
  eval(['Iz',num2str(i),'=','link.I(3,3)',';']);
  eval(['Ixy',num2str(i),'=','link.I(1,2)',';']);
  eval(['Ixz',num2str(i),'=','link.I(1,3)',';']);
  eval(['Iyz',num2str(i),'=','link.I(2,3)',';']);
  fv(i)=link.friction(qd(i));
end
gc=gc(3);
%生成系数矩阵
D=subs(D);
H=subs(H);
G=subs(G);
fv=fv(:);
end