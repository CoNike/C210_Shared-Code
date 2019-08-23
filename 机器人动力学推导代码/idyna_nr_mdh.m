function Tor=idyna_nr_mdh(robot,q,qd,qdd,gravity,f_end)
%����DHģ����ţ��ŷ�������㷨�Ļ������涯��ѧ��⡣
%Ŀǰֻ֧�����ֽ⡣
% Tor=idyna_nr_mdh(robot,q,qd,qdd,gravity,f_end)
%����robotΪ������ģ�ͣ��������˶�ѧ�����Ͷ���ѧ������
%����qΪ�ؽڽǶȣ�1XN����qdΪ�ؽڽǼ��ٶ�,qddΪ��Ӧ�ؽڽǼ��ٶȣ�
%gravityΪ�������ٶȣ�f_endΪĩ��ִ����ʩ���غɣ�
%�������ڲ�����ʱ��ѡ��Ĭ��ֵ��
%���TorqueΪ�ؽ�Ť�ء�
%�����ο���Peter,corke��RTB�е�rne������
%�ο��鼮Ϊ������ѧ����ģ�������Ӿ�7.7�ڡ�
%% ���úͻ�ȡ��������
n=robot.n; %������������Ŀ
z0=[0,0,1]'; %������ϵ��z0�ķ���
grav= robot.gravity; %�������ٶȣ���������
fend = zeros(6, 1);%ĩ��ʩ�ӵ��غ�,W=[fx Fy Fz Mx My Mz]';
debug1=1;%���õ��Բ���
debug2=1;
if nargin>4%�ж��Ƿ������������ٶ�
    grav=gravity;
end
if nargin==6%�ж��ⲿ�Ƿ�ʩ���غ�
    fend=f_end;
end
%% �ж������Ƿ����Ҫ��
if numcols(q)~=n||numcols(qd)~=n||numcols(qd)~=n||numrows(q)~=1||...
        numrows(qd)~=1||numrows(qdd)~= 1||length(fend)~=6
    error('������������')
end
%% ���
%��ʼ����������
R={};%�洢������ת����
Fm = [];%�洢�м������ݣ����Ĵ�
Nm = [];%�洢�м��������ݣ����Ĵ�
r_rela= [];%�洢��������֮�����
w = zeros(3,1);%��ʼ���ٶ�
wd = zeros(3,1);%��ʼ�Ǽ��ٶ�
vd = grav(:);%��ʼ���ٶȣ��������ٶ�
for i=1
    link=robot.links(i);
    Ti=link.A(q(i));
    switch link.type %����������������ȡ���˳���d��ֵ
        case 'R'
            d = link.d;
        case 'P'
            d = q(i);
    end
    alpha=link.alpha;%��ȡ����Ť��
    r_rela=[link.a; -d*sin(alpha); d*cos(alpha)];%����ϵi+1���������ϵi��λ��
    %�ͱ�׼DH������ͬ����Ϊ���궨�岻ͬ,���ұ任˳��ͬ
    if i==1
       Ti=robot.base*Ti; %���ǵ���ϵ��1ϵ����任
       r_rela= t2r(robot.base) * pm;%���ǵ���Ч�˶�ֻ�����˲��֣���Ȼλ���б仯
    end
    %�洢��ر������Ա����
    r_temp(:,i)=r_rela;
    R{i}=t2r(Ti);
end
%ΪʲôҪ�洢��ر����������ǽ��˴���������ѭ��һ������
%��Ϊ���������һ��ѭ���е�ĳЩ����ʱ�������ǰ��ѭ����i+1��Ӧ�����ݣ�
for i=1:n
    link=robot.links(i);%��ȡ���˽ṹ��
    Ri=R{i}';%�˴���Ҫת�ã�ת�ã�������������������i�����i+1�ı任��ʾ�����ģ����Ǻ���ȷ
    r_rela=r_temp(:,i);%��������֮���λ��
    r=link.r;%����������������˵�λ��
    switch link.type
        case 'R'
            w=Ri*w+qd(i)*z0;%�����ٶ�%�˴����в�û����z0��ʾ������i+1zi+1
            wd=Ri*wd+cross(Ri*w,qd(i)*z0)+qdd(i)*z0;%���Ǽ��ٶȣ��������й�ʽ��������
            %�ڶ������дΪ����X��������ʽ
            vd=Ri*(vd+cross(wd,r_rela)+cross(w,cross(w,r_rela)));%���ٶ����
        case 'P'
            w=Ri*w;
            wd=Ri*wd;
            vd=Ri*(vd+cross(wd,r_rela)+cross(w,cross(w,r_rela)))+...
                2*cross(w,qd(i)*z0)+qdd(i)*z0;
    end
    vd_cen=cross(wd,r)+cross(w,cross(w,r))+vd;%�������Ĵ��ļ��ٶ�
    F=link.m*vd_cen;%�������Ĵ�����
    N=link.I*wd+cross(w,link.I*w);%�������Ĵ�Ť��
    %�洢��������Ա����
    Fm=[Fm F];
    Nm=[Nm N];
end
%�������������ʱ����ע�͵�
if debug1==1
    fprintf('R:');disp(R)
    fprintf('Fm:');disp(Fm)
    fprintf('Nm:');disp(Nm)
    fprintf('\n');
end
%���ؽ�����
%��ȡ�ؽ���������
fend=fend(:);
ff=fend(1:3);
fn=fend(4:6);
for i=n:-1:1
    link=robot.links(i);%��ȡ���˽ṹ��
    r=link.r;%��ȡ��������λ��
    %���������������Ļ�ȡ�����й�ʽ��Ҫ������������ѭ����+1
    %������������Ϊ�Ľ�DH�Ķ��岻ͬ��
    if i==n
        Ri=eye(3);
        r_rela=[ 0 0 0]';
    else
        Ri=R{i+1};%��ȡ��ת���󣬲���Ҫת��
        r_rela=r_temp(:,i+1);
    end
    %ע��ff��fn�����˳���ܷ���
    fn=Ri*fn+Nm(:,i)+cross(r,Fm(:,i))+cross(r_rela,Ri*ff);%�ؽ�����
    ff=Ri*ff+Fm(:,i);%�ؽ���
    if debug2==1
        fprintf('fn:');disp(fn)
        fprintf('ff:');disp(ff)
        fprintf('\n');
    end
    switch link.type
        %���ںͱ�׼DHһ��������
        case 'R'
            Tor(i)=fn'*z0-link.friction(qd(i))+link.G^2 * link.Jm*qdd(i);
        case 'P'
            Tor(i)=ff'*z0-link.friction(qd(i))+link.G^2 * link.Jm*qdd(i);
    end
end
end