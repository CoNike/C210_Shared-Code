%����ָ�����Ļ��������˶�ѧ��ֵ���
%q=ikine_num_p(robot,Tg,q0)


function q=ikine_num_p(robot,Tg,q0)
%% ��֤�����Ƿ���ȷ
%�ж������ģ���Ƿ���ȷ
if ~isa(robot,'SerialManu')
    error('����ģ�Ͳ���')
end
n=robot.n;
%�ж�����Ƕ��Ƿ���ϱ�׼
if size(q0,2)~=n
    error('q must have %d column.',n);
end
%�ж�Tg�Ƿ����Ҫ��
if isa(Tg,'SE3')
    Tg=double(Tg);
elseif(size(Tg)~=[4,4])
    error('������λ�˾���')
end

flag1=0;%��ʾĩ�˾���
%% ���
flag=true;
dis1=10000;
k=0;
while flag
    k=k+1;% ȷ����������
    T0=robot.fkinep(q0);%q0ʱĩ��λ��
    
    %������λ�õ���άʸ��
    T=Tg/T0;
    v=T(1:3,4);
    R=T(1:3,1:3);
    w=iskew(R);
    delta=[v;w];
    jaco=robot.jacobp(q0,'d');%����ſ˱�
    %���Ƿ����������
    if n>6
        jaco_det=jaco*jaco';
    else
        jaco_det=jaco'*jaco;
    end
    det_j=sqrt(abs(det(jaco_det)));   %����ſ˱�����ʽ�ľ���ֵ
    
    %���Ƿ���������˵��Ż�
    if n>6
        %����det_j��ȡ��ͬ�ĵ�������
        if  det_j>10^-4
            temp=jaco'*inv(jaco*jaco')*delta;
            alpha=1;
        else
            %����λ�ø������õķ���
            lambda=0.01;
            temp=jaco'*inv(jaco*jaco'+lambda*eye(6))*delta;
            alpha=1.5;
        end
    elseif n<6
        if  det_j>10^-4
            temp=inv(jaco'*jaco)*jaco'*delta;
            alpha=1;
        else
            %����λ�ø������õķ���
            lambda=0.01;
            temp=inv(jaco'*jaco+lambda*eye(n))*jaco'*delta;
            alpha=1.5;
        end
    else
        if  det_j>10^-4
            temp=inv(jaco)*delta;
            alpha=1;
        else
            %����λ�ø������õķ���
            lambda=0.01;
            temp=jaco'*inv(jaco*jaco'+lambda*eye(6))*delta;
            alpha=1.5;
        end
    end
    %�������λ��ѡ���������
     if dis1>0.1
        beta=0.3;
    else
        beta=0.92;
    end
    q0=q0+alpha*temp'*beta;          %�����º�ĽǶ�
    %����Ƕȳ�������
    qlim=robot.qlim;
    for i=1:length(q0)
        if strcmp(robot.jolinks(i).jointtype,'R')
            if q0(i)>=qlim(i,2)*pi/180
                q0(i)=q0(i)-2*pi;
            elseif q0(i)<-pi
                q0(i)=q0(i)+2*pi;
            end
        end
    end
    %��¼����
    dis2=dis1;
    dis1=norm(delta);
    %�ж��Ƿ��������
    if (((abs(dis2-dis1))<10^-6)&& (dis1<10^-6))||(k>1000)        
        flag=false;
    end
    if flag1==1
        dis1
        q0
    end
end
q=q0;

    function w=iskew(R)
        w=[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)]/2;
    end
end