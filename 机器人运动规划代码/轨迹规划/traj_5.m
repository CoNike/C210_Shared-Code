function [q,qd,qdd]=traj_5(q0,qf,t,qd0,qdf,qdd0,qddf)
%jtraj函数，能够在q0和qf之间利用五次函数拟合一条轨迹；
%输入q0和qf为起始和最终关节角度，t为时间序列或者步数
%qd0和qdf为初始和最终速度，若不给出，则为0
%qdd0和qddf为初始和最终加速度，若不给出，则为0
%输出分别为角度序列q，速度序列qd，加速度序列qdd

%% 对于时间T的处理，生成序列
if length(t)==1
%     tscale=t;
    tv=(0:t-1)'/(t-1);
    tdis=1;
else
%     tscale=length(t);
    tmax=max(t);
    tmin=min(t);
    tdis=tmax-tmin;
    tv=(t-tmin)/(tdis);
    tv=tv(:);
end
%% 判断输入向量长度是否一致
q0 = q0(:);
qf = qf(:);
if length(q0)~=length(qf)
  error("输入矢量长度不一致")  
end
if nargin==3
    qd0=zeros(length(q0),1);
    qdf=qd0;
    qdd0=zeros(length(q0),1);
    qddf=qdd0;
elseif nargin==5
    qdd0=zeros(length(q0),1);
    qddf=qdd0;
    if (length(qd0)~=length(q0))||(length(qd0)~=length(qdf))
        error("输入矢量长度不一致")
    end
    qd0 = qd0(:);
    qdf = qdf(:);
elseif nargin==7
    qdd0 = qdd0(:);
    qddf = qddf(:);
    if (length(qdd0)~=length(q0))||(length(qdd0)~=length(qddf))
        error("输入矢量长度不一致")
    end
else
    error("输入参数数目不对")
end
%% 求解多项式系数
F=q0;
E=qd0;
D=qdd0/2;
%暂存
temp1=qf-D-E-F;
temp2=qdf-2*D-E;
temp3=qddf/2-D;
A=-2*(temp2-3*temp1)+(temp3-temp2);
B=temp2-3*temp1-2*A;
C=temp1-B-A;

%% 求解输出
%输出角度
tt = [tv.^5 tv.^4 tv.^3 tv.^2 tv ones(size(tv))]; %时间矩阵
coeff=[A B C D E F]';% 系数矩阵
q=tt*coeff;
%输出角速度
if nargout >= 2
    coeff = [ zeros(size(A)) 5*A 4*B 3*C  2*D E ]';
    qd = tt*coeff/tdis;
end
% 输出角加速度
if nargout >= 3
    coeff = [ zeros(size(A))  zeros(size(A)) 20*A 12*B 6*C  2*D]';
    qdd = tt*coeff/tdis^2;
end
end