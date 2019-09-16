%����ָ�����Ļ������ſ˱Ⱦ�������
%J=jacobp(robot,q,var)
%robotΪ������ģ�ͣ�ΪSerialLink��
%qΪ�����˹ؽڽǶ�
%var ���������������ſ˱Ⱦ�������Ͱ�����
%s,������ϵ�ռ��ſ˱Ⱦ������
%b����������ϵ�µ��ſ˱Ⱦ���
%d��������ϵ�µ��ſ˱Ⱦ���




function J=jacobp(robot,q,var)
%% �����ж���ѡ��
if isa(robot,'SerialLink')
    error('����ģ�Ͳ���')
end
n=robot.n;
%�ж�����Ƕ��Ƿ���ϱ�׼
if size(q,2)~=n
    error('q must have %d column.',n);
end
%% �ſ˱����
T=eye(4);
tw=robot.twist;
for i=1:n
    link=robot.jolinks(i);
    ad=adjoint(T);
    J(:,i)=ad*tw(i,:)';
    T=T*link.isom(q(i));
end
%% �ж����
if nargin==2
    argu='s';
else
    argu=var;
end
if strcmp(argu,'s')%�ռ��ſɱȾ���
    return
elseif strcmp(argu,'b') %��������ſɱȾ���
    Tq=T*robot.T0;
    J=adjoint(inv(Tq))*J;
elseif  strcmp(argu,'d') 
    Tq=T*robot.T0;
    J=adjoint(inv(Tq))*J;
    R=Tq(1:3,1:3);
    J=[R zeros(3,3);
        zeros(3,3) R]*J;
else
    error('������������')
end
end

function T6=adjoint(T4)
if size(T4)~=[4,4]
    error('���������4X4');
end
r=T4(1:3,1:3);
p=T4(1:3,4);
T6=[r skew0(p)*r;
    zeros(3,3) r];
end