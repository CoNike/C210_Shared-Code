%����ָ����ģ�͵Ļ�������������condition number�����
%connum=ConNumIndex(robot,q)
%robotΪ������ģ�ͣ�ΪSerialLink��
%qΪ�����˹ؽڽǶ�

%�ο�����Ϊ�����׵����ġ�������ѧ��
%2020.5.1������
function connum=ConNumIndex(robot,q)
if ~isa(robot,'SerialManu')
    error('����ģ�Ͳ���')
end
n=robot.n;
%�ж�����Ƕ��Ƿ���ϱ�׼
if size(q,2)~=n                         %�˴�ʹ��size��Ϊ���Ժ���չq�ӵ�һ�Ƕ��鵽�Ƕ�����
    error('q must have %d column.',n);
end
%����ſ˱Ⱦ����ſ˱Ⱦ����������Ҫȷ��
for i=1:size(q,1)
    jaco=robot.jacobp(q(i,:));
    [~,V,~]=svd(jaco);
    connum(i)=V(n,n)/V(1,1);
end
end