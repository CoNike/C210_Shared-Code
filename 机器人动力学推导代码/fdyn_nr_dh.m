function qdd=fdyn_nr_dh(robot,q,qd,torque,gravity,f_end)
%DHģ���£�����ţ��ŷ�����������Ļ��������˶�ѧ���
%Ŀǰֻ֧���������
%����robotΪ������ģ�ͣ��������˶�ѧ�����Ͷ���ѧ����
%����qΪ�ؽڽǶȣ�1XN����qdΪ�ؽڽǼ��ٶ�,torqueΪ�ؽ�Ť��
%gravityΪ�������ٶȣ�f_endΪĩ��ִ����ʩ���غɣ�
%�������ڲ�����ʱ��ѡ��Ĭ��ֵ
%���qddΪ��Ӧ�ؽڽǼ��ٶ�
%�����ο���Peter,corke��RTB�е�accel����
%�ο��鼮Ϊ������ѧ����ģ�滮�����7.6��
%% ���úͻ�ȡ��������
n=robot.n; %������������Ŀ
grav= robot.gravity; %�������ٶȣ���������
fend = zeros(6, 1);%ĩ��ʩ�ӵ��غ�,W=[fx Fy Fz Mx My Mz]';
debug1=0;%���õ��Բ���
if nargin>4%�ж��Ƿ������������ٶ�
    grav=gravity;
end
if nargin==6%�ж��ⲿ�Ƿ�ʩ���غ�
    fend=f_end;
end
%% �ж������Ƿ����Ҫ��
if numcols(q)~=n||numcols(qd)~=n||numcols(qd)~=n||numrows(q)~=1||...
        numrows(qd)~=1||numrows(torque)~= 1||length(fend)~=6
    error('������������')
end
%% ���
%Ԥ��B��q���������Ҫ�Ĳ���
qd_temp=zeros(1,n);
qdd_temp=eye(n);
%���B��q��
%�����Ҫ���ǻ�����DHģ���Ƿ�����˸Ľ��汾����torque�����ֵҲ��
for i=1:n
    if robot.mdh == 0
        M(i,:)=idyna_nr_dh(robot,q,qd_temp,qdd_temp(:,i),[0 0 0],fend); %�˴��Ƿ�Ӧ�ð���fend���ܴ���
    else
        M(i,:)=idyna_nr_mdh(robot,q,qd_temp,qdd_temp(:,i),[0 0 0],fend);%�˴��Ƿ�Ӧ�ð���fend���ܴ���
    end
end
%�����ٶ�
if robot.mdh == 0
    tor_temp = idyna_nr_dh(robot, q, qd, zeros(1,n),grav,fend);%��torque�����ֵ
else
    tor_temp = idyna_nr_mdh(robot, q, qd, zeros(1,n),grav,fend);%��torque�����ֵ
end
if debug1==1
    fprintf('M:');disp(M)
    fprintf('tor_temp:');disp(tor_temp)
    fprintf('\n');
end
qdd=M\(torque-tor_temp)';%�����ٶ�
end