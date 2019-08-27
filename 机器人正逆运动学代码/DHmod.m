function T=DHmod(alpha,a,d,theta,argument)
%���ڸĽ�DH���������������֮��ı任����
%argument������ȷ��alpha��theta���õ��ǻ����ƻ��ǽǶ���
if nargin==4
    argument='deg';
end
if strcmp(argument,'deg')
    T1=SE3.Rz(theta,'deg')*SE3(0,0,d);
    T2=SE3.Rx(alpha,'deg')*SE3(a,0,0);
elseif strcmp(argument,'rad')
    T1=SE3.Rz(theta)*SE3(0,0,d);
    T2=SE3.Rx(alpha)*SE3(a,0,0);
else
    error('�Ƕ���������')
end
T=T2*T1;
T=T.T;
end