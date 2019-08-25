function [q,qd,qdd]=tracj_3(q0,qf,t,qd0,qdf)
%jtraj函数，能够在q0和qf之间利用三次函数拟合一条轨迹；
%输入q0和qf为起始和最终关节角度，t为时间序列或者步数
%qd0和qdf为初始和最终速度，若不给出，则为0
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
elseif nargin==5
    if (length(qd0)~=length(q0))||(length(qd0)~=length(qdf))
        error("输入矢量长度不一致")
    end
    qd0 = qd0(:);
    qdf = qdf(:);
else
    error("输入参数数目不对")
end
%% 求解多项式系数
C=qd0;
D=q0;
temp1=qf-C-D;
temp2=qdf-C;
A=temp2-2*temp1;
B=temp1-A;

%% 求解输出
tt = [tv.^3 tv.^2 tv ones(size(tv))]; 
coeff=[A B C D]';
q=tt*coeff;
%输出速度
if nargout >= 2
    coeff = [ zeros(size(A)) 3*A 2*B C ]';
    qd = tt*coeff/tdis;
end
% 输出角加速度
if nargout >= 3
    coeff = [ zeros(size(A))  zeros(size(A)) 6*A 2*B]';
    qdd = tt*coeff/tdis^2;
end
end