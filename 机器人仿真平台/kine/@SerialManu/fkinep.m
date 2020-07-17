%�������˻���ָ������ʽ�����˶�ѧ
%T=fkinep(robot,q��var)
%����robotΪ����ָ���������Ļ�����ģ�ͣ�SerialLink����
%qΪ�ؽڽǶȣ�
%var��'rad'or 'deg'���ڹ涨�Ƕȵĵ�λ��
%TΪ4X4����

%ָ������ʽ�Ĳο����ף�
%�����׵ȡ�������ѧ����ģ�������Ӿ���
%������ȡ�������ѧ�ļ��λ�����

%creator:Huang Zhouzhou   Time:2019/9/6
%Huazhong University of Science and Technology
function T=fkinep(robot,q,var)
%% ��֤�����Ƿ���ȷ
%�ж������ģ���Ƿ���ȷ
if ~isa(robot,'SerialManu')
    error('����ģ�Ͳ���')
end
n=robot.n;
%�ж�����Ƕ��Ƿ���ϱ�׼
if size(q,2)~=n
    error('q must have %d column.',n);
end
%�ж�����Ƕȵĵ�λ��
angleunit='rad';
if nargin ==3
    angleunit=var;
end
if strcmp(angleunit,'deg')
    q=q*pi/180;
elseif ~strcmp(angleunit,'rad')
    error('�Ƕ���������')
end

%% ������˶�ѧ
T=robot.base;
for i=1:n
    link=robot.jolinks(i);
    T=T*link.isom(q(i));
end
T=T*robot.T0*robot.tool;
end