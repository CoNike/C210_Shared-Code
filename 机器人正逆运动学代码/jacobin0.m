function J=jacobin0(DH,q)
%�������˵��ſɱȾ�������ΪDH����DH�͹ؽڽǶ�q
%DH����Ϊ��׼DH������ģ,��Ե���ȫ��������ת�ؽڵĻ�����
%% �ж������Ƿ����Ҫ��
q=q(:);
[dhrow,dhcol]=size(DH);
if (dhcol~=4)||(dhrow~=length(q))
    error('����Ĳ���������Ҫ��')
end
%���ӻ����굽�ؽ�i����ı任����
T=cell(1,6);%����Ԫ������
alpha=DH(:,1);
a=DH(:,2);
d=DH(:,3);
qk=DH(:,4)+q;
DH=[alpha a d qk];
for i=1:dhrow
    T{i}=DHstd_sy(DH(i,1),DH(i,2),DH(i,3),DH(i,4));%����������˱任����
    if i>1
       T{i}=T{i-1}*T{i}; %��ǰһ�������ȡ0��i�ı任����
    end
end
%% ���ſ˱Ⱦ���
%Ԥ����ȡz0,pe
z0=[0 0 1]';
pe=T{6}*[0 0 0 1]';
pe=pe(1:3);

for i=1:dhrow
    %���pi��zi
    if i==1
        w=z0;
        p=[ 0 0 0]';
    else
        R=T{i-1}(1:3,1:3);
        w=R*z0;
        p=T{i-1}*[0 0 0 1]';
        p=p(1:3);
    end
    v=cross(w,pe-p);
    %�ϳ��ſ˱Ⱦ���
    J(:,i)=[v;w];
end
end