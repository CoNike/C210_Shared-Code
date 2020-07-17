function J=jacobi_screw(twist,T0,q,argument)
%利用旋量方法求解雅克比矩阵
%利用旋量twist，起始位置T0和关节角度q
%输入twist为一个6xN的矩阵，N为关节数，前三行轴线旋转的方向，
%后三行为轴线上一点；
%输入T0为末端执行器的起始位姿矩阵，q为关节角度，单位为弧度
%argument用于声明输出雅克比矩阵类型%：
%'s'为空间雅可比矩阵
%'b'为物体雅可比矩阵
%输出为雅克比矩阵J
%注意基于指数积方法与其他方法求解出的雅克比矩阵之间的差别！！！
%% 判断输入是否符合要求
[twrow,twcol]=size(twist);
if (twrow~=6)||(twcol~=length(q))
    error('输入的参数不规范')
end
argu='s';
if nargin==4
    argu=argument;
end
%% 利用旋量求变换矩阵
J=zeros(6,twcol);
T=eye(4);
for i=1:twrow
    tw=Twist('R',twist(1:3,i),twist(4:6,i));
    screw=SE3(T).Ad*tw.double';
    T_temp=tw.T(q(i));
    T=T*T_temp;
    J(:,i)=screw;
end
if strcmp(argu,'s')%空间雅可比矩阵
    return
elseif strcmp(argu,'b') %求解物体雅可比矩阵
    if isa(T0,'SE3')
        T0=T0.T;
    end
    Tq=T*T0;
    J=SE3(inv(Tq)).Ad*J;
end
end