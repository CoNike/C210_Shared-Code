function T=tracj_l(T0,Tg,t,argument)
%������λ��֮�����ɾ��ȵ�ֱ�߲�ֵ
%����T0��Tg����4*4�ĸ���任����tΪʱ�����л��߲���
%���TΪ������rpy�Ǻ�λ�����У�Nx6��
%argumrnt����������ת����ı�ʾ���ͣ����а���
%      'rpy'��rpy�Ǳ�����ת����
%      'angvec'����ת��ͽǶȵ���ʽ������ת����
%      'eul'��ŷ���Ǳ�ʾ��ת����
%       �������룬��Ĭ�ϰ�rpy

%% ��׼��t���ж�t�Ƿ���ϱ�׼
%�����ֲ�ͬ���͵�t��ת��Ϊ0~1������
if length(t)==1
    ts=(0:t-1)/(t-1);
else
    tmax=max(t);
    tmin=min(t);
    if tmin<=0
        error('����ʱ�����в�����Ҫ��')
    end 
    ts=(t-tmin)/(tmax-tmin);
end
%�ж��Ƿ�����Ƕ���������û����rpy����
if nargin==3
    argument='rpy';
elseif ~ischar(argument)
    error('������������һ������')
end
%% ����ת��
%��ȡ��ʼ��ת�����ƽ������
R0=t2r(T0);
Rg=t2r(Tg);

t0=transl(T0);
tg=transl(Tg);
%�����Ա任����
R=Rg*inv(R0);
%Ԥ�����������С
rpy=zeros(length(t),3);
if strcmp(argument,'rpy')  %�ж�char�Ƿ���ϵĺ���
    rpy0=tr2rpy(R); %����ת����ת��Ϊrpy
    for i=1:length(ts)
        rpy_temp=rpy0*ts(i); %���м����е�rpy
        R_temp=rpy2r(rpy_temp)*R0;%���м����е���ת����
        rpy_temp2=tr2rpy(R_temp);%����Ӧ��rpy
        rpy(i,:)=rpy_temp2(:)';
    end
elseif strcmp(argument,'angvec')
    [theta,vector]=tr2angvec(R);%������ת��Ϊ�����ʽ
    for i=1:length(ts)
        theta_temp=theta*ts(i);%����м�Ƕ�
        R_temp=angvec2r(theta_temp,vector)*R0; %����м����
        rpy_temp2=tr2rpy(R_temp);%ת��Ϊrpy
        rpy(i,:)=rpy_temp2(:)';
    end
elseif  strcmp(argument,'eul')
    eul0=tr2eul(R);%������ת��Ϊŷ����
    for i=1:length(ts)
        rpy_temp=eul0*ts(i);%����м�ŷ����
        R_temp=rpy2r(rpy_temp)*R0;%ת��Ϊ�м����
        rpy_temp2=tr2rpy(R_temp);%ת��Ϊrpy
        rpy(i,:)=rpy_temp2(:)';
    end
else     %�ж��Ƿ������������
    error('��������ʵĽǶȱ������')
end

%% �����������
t=ts'*(tg-t0)'+t0';
T=[rpy t];
end