function T=circle_pose(p1,p2,p3,p0,t)
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
%% ���
[r0,~]=circle_r(p1,p2,p3);%���Բ�ι켣Բ��
p=circle_jtraj(p1,p2,p3,t);%���Բ�ι켣�Ĺ켣��
M=eye(4,4);%Ԥ�����

for i=1:t
    z=p0-p(i,:)'; %ָ��Ŀ���ķ���
    p0r0=p0-r0;
    x=cross(p0r0,z);% p0p(i)r0��ƽ�淨����
    %����޷����x
    if norm(x)<10^-10
        x=T{i-1}(1:3,4);
    end
    if i>1&&x'*x0<0%�ж�����x�Ƿ�����
        x=-x;
    end
    x0=x;
    y=cross(z,x);%����y����
    %p(i)���λ�˾���
    M(1:3,1)=x/norm(x);
    M(1:3,2)=y/norm(y);
    M(1:3,3)=z/norm(z);
    M(1:3,4)=p(i,:)';
    T{i}=M;
end
end