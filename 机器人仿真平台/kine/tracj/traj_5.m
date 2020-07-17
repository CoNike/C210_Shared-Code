function [q,qd,qdd]=traj_5(q0,qf,t,qd0,qdf,qdd0,qddf)
%jtraj�������ܹ���q0��qf֮��������κ������һ���켣��
%����q0��qfΪ��ʼ�����չؽڽǶȣ�tΪʱ�����л��߲���
%qd0��qdfΪ��ʼ�������ٶȣ�������������Ϊ0
%qdd0��qddfΪ��ʼ�����ռ��ٶȣ�������������Ϊ0
%����ֱ�Ϊ�Ƕ�����q���ٶ�����qd�����ٶ�����qdd

%% ����ʱ��T�Ĵ�����������
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
%% �ж��������������Ƿ�һ��
q0 = q0(:);
qf = qf(:);
if length(q0)~=length(qf)
  error("����ʸ�����Ȳ�һ��")  
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
        error("����ʸ�����Ȳ�һ��")
    end
    qd0 = qd0(:);
    qdf = qdf(:);
elseif nargin==7
    qdd0 = qdd0(:);
    qddf = qddf(:);
    if (length(qdd0)~=length(q0))||(length(qdd0)~=length(qddf))
        error("����ʸ�����Ȳ�һ��")
    end
else
    error("���������Ŀ����")
end
%% ������ʽϵ��
F=q0;
E=qd0;
D=qdd0/2;
%�ݴ�
temp1=qf-D-E-F;
temp2=qdf-2*D-E;
temp3=qddf/2-D;
A=-2*(temp2-3*temp1)+(temp3-temp2);
B=temp2-3*temp1-2*A;
C=temp1-B-A;

%% ������
%����Ƕ�
tt = [tv.^5 tv.^4 tv.^3 tv.^2 tv ones(size(tv))]; %ʱ�����
coeff=[A B C D E F]';% ϵ������
q=tt*coeff;
%������ٶ�
if nargout >= 2
    coeff = [ zeros(size(A)) 5*A 4*B 3*C  2*D E ]';
    qd = tt*coeff/tdis;
end
% ����Ǽ��ٶ�
if nargout >= 3
    coeff = [ zeros(size(A))  zeros(size(A)) 20*A 12*B 6*C  2*D]';
    qdd = tt*coeff/tdis^2;
end
end