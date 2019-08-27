function [D,H,G,fv,fc]=dyn_coef(robot,q,qd)
%此函数用拉格朗日动力学方法求解机器人动力学，主要用于测试拉格朗日动力学算法的正确性
%求解使用数值结果，目前只考虑旋转关节,不考虑电机的影响
% [D,H,G,fv,fc]=dyn_coef(robot,q,qd)
%robot为机器人模型，包含有机器人运动学和动力学相关参数
%q为机器人关节角度，qd为机器人关节角速度
%输出D为加速度项的系数矩阵，表征的是加速度对关节力的影响；
%输出H为速度项的系数矩阵，表征的是速度对各关节力的影响；
%输出G为重力项
%fv和fc为选择输出项，有待确定和标定
%输出fv为各关节的粘滞摩擦力
%输出fc为各关节的库伦摩擦力
%参考文献为熊有伦等著的《机器人学：建模、控制与视觉》和
%布鲁诺·西西里安诺等著的《机器人学：建模规划与控制》相关部分
%程序参考：ttps://www.jianshu.com/p/6d04539f1cfe
%% 判断输入是否正确
n=robot.n;%获取连杆数目
q=q(:);
qd=qd(:);
if length(q)~=n||length(qd)~=n
    error('error data!!')
end
%% 设置参数
grav=robot.gravity;%设置重力加速度
 %获取连杆参数
for i=1:n
    eval(['syms ','q',num2str(i),' real;']);            %设置关节变量
    eval(['syms ','dq',num2str(i),' real;']);          %设置关节速度
    qk(i)=eval(['q',num2str(i)]);
    link=robot.links(i);
    mass(i)=link.m;
    inertia{i}=link.I;
    r_cen{i}=link.r;
end
a=robot.a;
d=robot.d;
alpha=robot.alpha;
offset=robot.offset+qk;%qk在此处添加能够简化运算量
dhtable=[alpha' a' d' offset'];
%% 求解关节0到i的变换矩阵
%此处采用的事标准DH模型
for i=1:n
    if i==1    %0-1的变换时
        T_temp=dh_std_sy(dhtable(i,1),dhtable(i,2),dhtable(i,3),dhtable(i,4));%生成变换矩阵
        T{i}=simplify(T_temp);                                        %必须简化
    else       % 0-i的变换时需要进行迭代相乘
        T_temp=dh_std_sy(dhtable(i,1),dhtable(i,2),dhtable(i,3),dhtable(i,4));
        T_temp=T{i-1}*T_temp;                                          %迭代过程
        T{i}=simplify(T_temp);
    end
end
%% 求解伪惯性矩阵
J={};
for i=1:n
    I=par_axis(inertia{i},r_cen{i},mass(i));           %关节原点处惯性矩阵
    J{i}=JMatrix(I,r_cen{i},mass(i));                  %生成为惯性矩阵
end
%% 求解D(q)
for i=1:n
    for j=i:n %由于是对称矩阵，可以只求上三角部分
        d=0;%每次求解时更新0
        %求解D(i,j)
        for k=max(i,j):n 
            d=d+trace(eval(['diff(T{k},q',num2str(i),')'])*J{k}...
                *eval(['diff(transpose(T{k}),q',num2str(j),')']));
        end
        D(i,j)=simplify(d);
        D(j,i)=D(i,j);
    end
end
%% 求解H(q,dq)
for i = 1:n
    for j = 1:n
        h = 0;
        %求解H
        for k = 1:n
            h=h+1/2*(eval(['diff(D(i,j),q',num2str(k),')'])+...
                eval(['diff(D(i,k),q',num2str(j),')'])-...
                eval(['diff(D(j,k),q',num2str(i),')']))*eval(['dq',num2str(k)]);
        end
       H(i,j) = simplify(h);
    end
end
%% 求解G(q)
g_vec=[-grav;0];
for i = 1:n
    g = 0;
    for j = i:n
        g=g-mass(j)*g_vec'*eval(['diff(T{j},q',num2str(i),')'])*[r_cen{j} 1]';
    end
    G(i) = simplify(g);
end
G = G(:);
%% 赋值输出
for i=1:n
    eval(['q',num2str(i),' =q(i);']);            %设置关节变量
    eval(['dq',num2str(i),'=qd(i);']);          %设置关节速度
end
D=subs(D);
H=subs(H);
G=subs(G);
%% 求解fv
if nargout>=4
    for i=1:n
        link=robot.links(i);
      fv(i)=link.friction(qd);
    end
    fv=fv(:);
end
%% 求解fc
%目前机器人还未包含该系数
if nargout==5
    for i=1:n
        eval(['syms',' Fc',num2str(i),' real;']);   %设置Fc
        fc(i)=eval(['sign(dq',num2str(i),');']);
    end
    fc=fc(:);
end
end


%% 需要调用的函数
function T = dh_std_sy(alpha,a,d,theta)
%此函数利用标准DH参数求相邻连杆的变换关系
% 为符号运算
%输入为连杆参数
%输出为符号矩阵
T = [cos(theta),   -sin(theta)*cos(alpha),  sin(alpha)*sin(theta),  a*cos(theta);
    sin(theta),     cos(theta)*cos(alpha), -sin(alpha)*cos(theta),  a*sin(theta);
    0,                         sin(alpha),             cos(alpha),             d;
    0,                                  0,                      0,            1];
end

function I=par_axis(I0,r,m)
%此函数为平行轴定理的实现
%输入I0为初始矩阵，r为初始点指向的向量,m为物体质量
%输出为终点处的惯性矩阵
r=r(:);   %确保为列向量
if numrows(I0)~=3 && numcols(I0)~=3 && length(r)~=3
    error('输入数据维数不对')
end
I=I0+m*(r'*r*eye(3)-r*r');  %平行轴定理
end

function J = JMatrix(I,r,m)
%此函数用于生成伪惯性矩阵，其定义参考熊有伦等著《机器人学：建模控制与视觉》7.2.2节内容。
%输入I为惯性矩阵，r为质心相对于原点位置向量，m为质量
%输出J为伪惯性矩阵
if numrows(I)~=3 && numcols(I)~=3 && length(r)~=3
    error('输入数据维数不对')
end
r=r(:);
k=(I(1,1)+I(2,2)+I(3,3))/2;
I_temp=-I+diag([k,k,k]);
J=[I_temp m*r;
    m*r' m];
end