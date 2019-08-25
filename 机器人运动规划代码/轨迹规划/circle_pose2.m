function T=circle_pose2(p1,p2,p3,p0,t,R)
%�˺�����������ָ��ָ����켣��λ����������
%p1,p2,p3Ϊ�켣�����㣬p0Ϊָ����Ŀ��㣬tΪ��ֵ��Ŀ
%���TΪһϵ��������λ��
%% ����ֵ�������ж�
p1=p1(:);
p2=p2(:);
p3=p3(:);
p0=p0(:);
if length(p1)~=3||length(p2)~=3||length(p3)~=3||...
        length(p0)~=3
    error('�������ݸ�ʽ����')
end
z1=p0-p1;
z0=R(:,3);
if norm(cross(z1,z0))>10^-6
    error('�����ʼ��̬������')
end

%% ���
% [r0,~]=circle_r(p1,p2,p3);%���Բ�ι켣Բ��
n=nor_vec_p(p1,p2,p3);
if n'*z1<0
    n=-n;
end
x0=cross(z0,n);
y0=cross(z0,x0);
p=circle_jtraj(p1,p2,p3,t);%���Բ�ι켣�Ĺ켣��
M=eye(4,4);%Ԥ�����
c0=R(:,1)'*x0;
s0=R(:,1)'*y0;
q=atan2(s0,c0);
for i=1:t
    z=p0-p(i,:)'; %ָ��Ŀ���ķ���
    x=cross(z,n);% p0p(i)r0��ƽ�淨����
    if i>1&&x'*x0<0%�ж�����x�Ƿ�����
        x=-x;
    end
    x0=x;
    y=cross(z,x);%����y����
    %p(i)���λ�˾���
    M(1:3,1)=x/norm(x);
    M(1:3,2)=y/norm(y);
    M(1:3,3)=z/norm(z);
    M(1:3,1:3)=M(1:3,1:3)*rotz(q);
    M(1:3,4)=p(i,:)';
    T{i}=M;
end
end
