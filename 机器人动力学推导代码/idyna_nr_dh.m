function Tor=idyna_nr_dh(robot,q,qd,qdd,gravity,f_end)
%����DHģ����ţ��ŷ�������㷨�Ļ������涯��ѧ���
%Ŀǰֻ֧�����ֽ�
%Tor=idyna_nr_dh(robot,q,qd,qdd,gravity,f_end)
%����robotΪ������ģ�ͣ��������˶�ѧ�����Ͷ���ѧ����
%����qΪ�ؽڽǶȣ�1XN����qdΪ�ؽڽǼ��ٶ�,qddΪ��Ӧ�ؽڽǼ��ٶ�
%gravityΪ�������ٶȣ�f_endΪĩ��ִ����ʩ���غɣ�
%�������ڲ�����ʱ��ѡ��Ĭ��ֵ
%���TorqueΪ�ؽ�Ť��
%�����ο���Peter,corke��RTB�е�rne����
%�ο��鼮Ϊ������ѧ����ģ�滮�����7.6��
%% ���úͻ�ȡ��������
%ȷ������Ƕȣ����ٶȣ��Ǽ��ٶ�Ϊ������
q=q(:)';
qd=qd(:)';
qdd=qdd(:)';
n=robot.n; %������������Ŀ
z0=[0,0,1]'; %������ϵ��z0�ķ���
grav= robot.gravity; %�������ٶȣ���������
fend = zeros(6, 1);%ĩ��ʩ�ӵ��غ�,W=[fx Fy Fz Mx My Mz]';
debug1=0;%���õ��Բ���
debug2=0;
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
%�����ؾ���Ͳ���
R={};

for i=1:n
    link=robot.links(i);%��ȡ���ˣ��ṹ�壩
    Ti=link.A(q(i));%��ȡ��������֮��ı任����
    switch link.type %����������������ȡ���˳���d��ֵ
        case 'R'
            d = link.d;
        case 'P'
            d = q(i);
    end
    alpha=link.alpha;%��ȡ����Ť��
    r_rela=[link.a; d*sin(alpha); d*cos(alpha)];%��ȡ��������ϵi+1�������������ϵi
    %�����λ����i+1ϵ�еı�ʾ���Ƿ���Ի�ȡλ��Ȼ��ֱ���������任��R0,i'*(Oi-Oi-1)��
    r_temp(i,:)=r_rela;%�ݴ����λ�����ݷ����������
    R{i}=Ti.t2r;%�洢��������֮�����ת�任���󷽱����
end
%���ֵ�ĳ�ʼ��
Tor=zeros(1,n);
%�洢Ť�غͼ��ٶ�
Fm = [];
Nm = [];
%����ֵ�ĳ�ʼ��
Rb = t2r(robot.base)';%��ȡ���˵�base�ı任�������ת����
w = Rb*zeros(3,1);%��ʼ�����˻������ٶ� %Ϊɶ��ֱ����zeros(3,1)��ʼ��
wd = Rb*zeros(3,1);%��ʼ�����˻����ĽǼ��ٶ�%Ҳ��ͬ��������
vd = Rb*grav(:);%��ʼ�������ļ��ٶ�
%�ܷ��þ����������ݽ��д���


%������Ƽ����ٶȣ����ٶȣ��Ǽ��ٶ�
for i=1:n
    link=robot.links(i);%��ȡ���˵Ľṹ��
    r_rela=r_temp(i,:)';%��ȡ��������֮�������
    Ri=R{i}';%��ȡ���ڵı任����,�ǵ�ת����ΪҪ��ȡ���������
    r=link.r';%����C�������������ϵ��λ������,ע��Ϊ����������������
    switch link.type
        case 'R' %��ת�ؽ�
            %ע����ٶȽǼ��ٶȵ�˳����Ϊ�Ǽ��ٶȻ����֮ǰ�Ľ��ٶ�
            wd=Ri*(wd+qdd(i)*z0+qd(i)*cross(w,z0));%���Ǽ��ٶ� 
            w=Ri*(w+qd(i)*z0); %�����ٶ�
            vd=Ri*vd+cross(wd,r_rela)+cross(w,cross(w,r_rela)); %�����ٶ�
        case 'P'
            wd=Ri*wd;
            w=Ri*w;
            vd=Ri*(vd+qdd(i)*z0)+2*qd(i)*cross(w,Ri*z0)...
                +cross(wd,r_rela)+cross(w,cross(w,r_rela));
    end
    vd_cen=vd+cross(wd,r)+cross(w,cross(w,r));%������ĵļ��ٶ�
    F=link.m*vd_cen;%�ɼ��ٶȲ�������
    N=link.I*wd+cross(w,link.I*w);%����ת��������������
    %�洢�������
    Fm=[Fm F];
    Nm=[Nm N];
end
if debug1==1
    fprintf('Fm:');disp(Fm)
    fprintf('Nm:');disp(Nm)
    fprintf('\n');
end
%�������ؽ���
%��ȡĩ����������
f_end=fend(:);
ff=f_end(1:3);%ĩ����
fn=f_end(4:6);%ĩ������
for i=n:-1:1
    link=robot.links(i);%��ȡ���˽ṹ��
    r_rela=r_temp(i,:)';%��ȡ��������֮�������
    r=link.r';%��ȡ������������˵Ĳ���
    %��ȡ�������˱任����
    if i==n
        Ri=eye(3);%i=nʱ,��Ӧ�����˾���ΪRn,n,��Ϊ��λ��
    else
        Ri=R{i+1};%i<nʱ,��Ӧ�����˾���ΪRn,n+1
    end
    %ע�����˳��
    %fn=cross(r_rela+r,Fm(:,i))+Ri*(fn+cross(Ri'*r_rela,ff))+Nm(:,i);%����ؽ��������
    fn=cross(r_rela+r,Fm(:,i))+Ri*(fn)+cross(r_rela,Ri*ff)+Nm(:,i);%����ؽ��������
    ff=Ri*ff+Fm(:,i);%����ؽ������
    if debug2==1
    fprintf('ff:');disp(ff)
    fprintf('fn:');disp(fn)
    fprintf('\n');
    end
    Ri=R{i};
    switch link.type
        %Ϊʲô��Ҫ��R������Ϊ�ؽ�i������ϵi-1��Ӧ
        case 'R'
%             t=fn'*(Ri'*z0); %��֤��������
            t=fn'*(Ri'*z0)-link.friction(qd(i))+link.G^2 * link.Jm*qdd(i);%�ؽ�����
            %��û�п��ǵ��������ת��������ת�ٵ��໥Ӱ��,��ֻ����ճ��Ħ���������������
        case 'P'
            t=ff*(Ri'*z0)-link.friction(qd(i))+link.G^2 * link.Jm*qdd(i);
            %�����ת�ؽ���⣬��֪�˴����ʱ���ת�ٵ��������Ҫ���ǵ�
    end
    Tor(i)=t;%�������
end
    %% ��������
    R1 = R{1};
    fn = R1*(fn);
    ff= R1*ff;
    wbase = [ff; fn];
end