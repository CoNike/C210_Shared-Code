%����ָ����ģ�͵Ļ�����ȫ�ֿɲ����ȣ�global manipulability index,GMI�����
%gmi=GMI(robot,q)
%robotΪ������ģ�ͣ�ΪSerialLink��
%qΪ�����˹ؽڽǶ����л��߽Ƕȿռ�켣����

%�ο�����Ϊ�����׵����ġ�������ѧ��
%2020.5.1 ������
function gmi=GMI(robot,q,state,q0)
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
if strcmp(state,'q')
    %�Ƕȹ켣�������
    
    %�ж�����Ƕ��Ƿ���ϱ�׼
    if size(q,2)~=n
        error('q must have %d column.',n);
    end
    A=0;
    B=0;
    for i=1:size(q,1)-1
        %����ſ˱Ⱦ����ſ˱Ⱦ����������Ҫȷ��
        jaco_manu=robot.ManiIndex(q(i,:));
        A=A+jaco_manu*jaco_manu*delta_q(q(i,:),q(i+1,:));
        B=B+jaco_manu*delta_q(q(i,:),q(i+1,:));
    end
    gmi=A/B;
elseif strcmp(state,'T')
    if size(q,2)~=6
        error('�����ʽ����')
    end
    for i=1:size(q,1)
        T=transl(q(i,1),q(i,2),q(i,3))*rpy2tr(q(i,4:6)*pi/180);
        qt(i,:)=robot.ikine_num_p(T,q0);
        q0=qt(i,:);
    end 
    A=0;
    B=0;
    for i=1:size(q,1)-1
        %����ſ˱Ⱦ����ſ˱Ⱦ����������Ҫȷ��
        jaco_manu=robot.ManiIndex(q(i,:));
        A=A+jaco_manu*jaco_manu*delta_q(q(i,:),q(i+1,:));
        B=B+jaco_manu*delta_q(q(i,:),q(i+1,:));
    end
    gmi=A/B;
else
    error('������������')
end

    function delta=delta_q(q1,q2)
        if length(q1)~=length(q2)
            error('����Ƕȵĳ��Ȳ�һ��')
        end
        iter=length(q1);
        delta=1;
        for ii=1:iter
            delta=delta*(q2(ii)-q1(ii));
        end
    end

end