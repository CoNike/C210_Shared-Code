function J=jacobi_screw(twist,T0,q,argument)
%����������������ſ˱Ⱦ���
%��������twist����ʼλ��T0�͹ؽڽǶ�q
%����twistΪһ��6xN�ľ���NΪ�ؽ�����ǰ����������ת�ķ���
%������Ϊ������һ�㣻
%����T0Ϊĩ��ִ��������ʼλ�˾���qΪ�ؽڽǶȣ���λΪ����
%argument������������ſ˱Ⱦ�������%��
%'s'Ϊ�ռ��ſɱȾ���
%'b'Ϊ�����ſɱȾ���
%���Ϊ�ſ˱Ⱦ���J
%ע�����ָ�������������������������ſ˱Ⱦ���֮��Ĳ�𣡣���
%% �ж������Ƿ����Ҫ��
[twrow,twcol]=size(twist);
if (twrow~=6)||(twcol~=length(q))
    error('����Ĳ������淶')
end
argu='s';
if nargin==4
    argu=argument;
end
%% ����������任����
J=zeros(6,twcol);
T=eye(4);
for i=1:twrow
    tw=Twist('R',twist(1:3,i),twist(4:6,i));
    screw=SE3(T).Ad*tw.double';
    T_temp=tw.T(q(i));
    T=T*T_temp;
    J(:,i)=screw;
end
if strcmp(argu,'s')%�ռ��ſɱȾ���
    return
elseif strcmp(argu,'b') %��������ſɱȾ���
    if isa(T0,'SE3')
        T0=T0.T;
    end
    Tq=T*T0;
    J=SE3(inv(Tq)).Ad*J;
end
end