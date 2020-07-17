%����ָ����ģ�͵Ļ����˿ɲ����ȣ�manipulability�����
%manipulability=ManiIndex(robot,q)
%robotΪ������ģ�ͣ�ΪSerialLink��
%qΪ�����˹ؽڽǶ�

%�ο�����Ϊ�����׵����ġ�������ѧ��
%2020.4.30 ������
function manipulability=ManiIndex(robot,q)
if ~isa(robot,'SerialManu')
    error('����ģ�Ͳ���')
end
n=robot.n;
%�ж�����Ƕ��Ƿ���ϱ�׼
if size(q,2)~=n                         %�˴�ʹ��size��Ϊ���Ժ���չq�ӵ�һ�Ƕ��鵽�Ƕ�����
    error('q must have %d column.',n);
end
%����ſ˱Ⱦ����ſ˱Ⱦ����������Ҫȷ��
jaco=robot.jacobp(q);

if n>=6  
    jaco_temp=jaco*jaco';
    manipulability=sqrt(abs(det(jaco_temp)));
else  %Ƿ���ɶȻ����˵Ŀɲ�������Ҫ����ȷ֤
    jaco_temp=jaco'*jaco;
    manipulability=sqrt(abs(det(jaco_temp)));
end
end