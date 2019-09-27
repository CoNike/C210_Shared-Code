% ikine_num
function [q,k,dis]=ikine_num(robot,Tg,q0)
%�������˵���ֵ��,ֻ�����ݵ����������һ��⣻
%����޷�������н⣬��ʱ��Ҳ�޷�����Լ���Ҫ�Ľ⡣
%��Ϊ���԰汾���ó����а���������Ĳ�����
%����ΪrobotΪ�����˽ṹ�壬���������˵��˶�ѧ������
%TgΪ����ξ������Ŀ��λ�ˣ�Ϊ4X4double����SE3
%q0Ϊ��������ʼ�㣬��������룬��Ϊ[0 0 0 0 0 0];
n=robot.n; %��ȡ������������
%�趨��ʼ�Ƕ�
qi=zeros(1,6);
if nargin==3
    qi=q0;
end
if length(qi)~=n
    error('����Ƕ�����')
end
%ת��TgΪ4X4double��������ֲ����������
if isa(Tg,'SE3')
    Tg=Tg.T;
end
k=1;
flag=true;%����ѭ����־
dis2=1;
while flag
    T0=robot.fkine(qi);%����ʼ״̬ʱλ�˾���
    T0=T0.T;
    T=Tg*inv(T0);
    v=T(1:3,4);         %��������任��Ӧ��ƽ����
    w=vex(t2r(T)-eye(3)); %����������ת����ת��Ϊ��ǵĳ˻�
    delta=[v;w];        %�ϲ�Ϊ6άʸ��
    dis1=norm(delta);    %��deltaģ����Ϊ�ж�ֵ
    jaco=robot.jacob0(qi);  %��������q0ʱ���ſ˱Ⱦ���
    det_j=abs(det(jaco));   %����ſ˱�����ʽ�ľ���ֵ
     %����det_j��ȡ��ͬ�ĵ�������
    if  det_j>10^-4
        temp=inv(jaco)*delta;
        alpha=1;
    else
        %����λ�ø������õķ���
        lambda=0.01;
        temp=jaco'*inv(jaco*jaco'+lambda*eye(6))*delta;
        alpha=1.5;
    end
    if dis1>0.1
        beta=0.3;
    else
        beta=0.92;
    end
    qi=qi+alpha*temp'*beta;          %�����º�ĽǶ�
    for i=1:length(qi)
        if qi(i)>=pi
            qi(i)=qi(i)-2*pi;
        elseif qi(i)<-pi
            qi(i)=qi(i)+2*pi;
        end
            
    end
    if ((abs(dis2-dis1))<10^-6)||(k>1000)        %�ж��Ƿ��������
        flag=false;
    end
    dis2=dis1;
    if nargout>1
    dis(k)=dis2;
    k=k+1;
    end
end
q=qi;
% plot(dis)
end