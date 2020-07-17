%����ָ����ģ�͵Ļ�����ȫ����������global manipulability index,GMI�����
%gmi=GMI(robot,q)
%robotΪ������ģ�ͣ�ΪSerialLink��
%qΪ�����˹ؽڽǶ����л��߽Ƕȿռ�켣����

%�ο�����Ϊ�����׵����ġ�������ѧ��
%2020.5.1 ������
function gci=GCI(robot,q,state,q0)
if ~isa(robot,'SerialManu')
    error('����ģ�Ͳ���')
end
if nargin==2
    state='q';
elseif nargin==3 %��Ϊĩ�˹켣���Ҳ�����ο��Ƕ�����ȷ�����˶�ѧ����Ĭ��Ϊ�����˳�ʼ�Ƕ�
    q0=robot.offset;
elseif nargin==4
    if strcmp(state,'T')
        error('����������Ŀ�겻ƥ��')
    end
end
n=robot.n;
%% ���
gci=0;
if strcmp(state,'q')
    %�Ƕȹ켣�������
    
    %�ж�����Ƕ��Ƿ���ϱ�׼
    if size(q,2)~=n
        error('q must have %d column.',n);
    end
    
    for i=1:size(q,1)
        gci=gci+robot.ConNumIndex(q(i,:));
    end
    gci=gci/size(q,1);
elseif strcmp(state,'T')
    if size(q,2)~=6
        error('�����ʽ����')
    end
    for i=1:size(q,1)
        T=transl(q(i,1),q(i,2),q(i,3))*rpy2tr(q(i,4:6)*pi/180);
        qtemp=robot.ikine_num_p(T,q0);
        gci=gci+robot.ConNumIdex(qtemp);
        q0=qtemp;
    end
    gci=gci/size(q,1);
else
    error('������������')
end

end