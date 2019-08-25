function [q,qd,qdd]=tracj_3(q0,qf,t,qd0,qdf)
%jtraj�������ܹ���q0��qf֮���������κ������һ���켣��
%����q0��qfΪ��ʼ�����չؽڽǶȣ�tΪʱ�����л��߲���
%qd0��qdfΪ��ʼ�������ٶȣ�������������Ϊ0
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
elseif nargin==5
    if (length(qd0)~=length(q0))||(length(qd0)~=length(qdf))
        error("����ʸ�����Ȳ�һ��")
    end
    qd0 = qd0(:);
    qdf = qdf(:);
else
    error("���������Ŀ����")
end
%% ������ʽϵ��
C=qd0;
D=q0;
temp1=qf-C-D;
temp2=qdf-C;
A=temp2-2*temp1;
B=temp1-A;

%% ������
tt = [tv.^3 tv.^2 tv ones(size(tv))]; 
coeff=[A B C D]';
q=tt*coeff;
%����ٶ�
if nargout >= 2
    coeff = [ zeros(size(A)) 3*A 2*B C ]';
    qd = tt*coeff/tdis;
end
% ����Ǽ��ٶ�
if nargout >= 3
    coeff = [ zeros(size(A))  zeros(size(A)) 6*A 2*B]';
    qdd = tt*coeff/tdis^2;
end
end