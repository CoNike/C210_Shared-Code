function [D,H,G,fv,fc]=dyn_lagr_sym(dhtable)
%�˺������������ն���ѧ�����������˶���ѧ
%����ʹ�÷��ű�Ŀǰֻ������ת�ؽ�,�����ǵ����Ӱ��
%[D,H,G,fv,fc]=dyn_lagr_sym(dhtable)
%����dhtableΪ�����˵�DH������Ĭ��Ϊ��׼DH���ҹؽڱ���Ϊ0
%����������m����������r_m,����ת������Im���ڳ���������
%����Ħ��Fc��ճ��Ħ��ϵ��FvҲ�ڳ���������,�Ҳ����ǹؽڼ�����
%�����ؽڽǶ�q,�Ǽ��ٶ�qd,�Ǽ��ٶ�qddҲ�ڳ���������
%���DΪ���ٶ����ϵ�����󣬱������Ǽ��ٶȶԹؽ�����Ӱ�죻
%���HΪ�ٶ����ϵ�����󣬱��������ٶȶԸ��ؽ�����Ӱ�죻
%���GΪ������
%fv��fcΪѡ�������
%���fvΪ���ؽڵ�ճ��Ħ����
%���fcΪ���ؽڵĿ���Ħ����
%�ο�����Ϊ�����׵����ġ�������ѧ����ģ���������Ӿ�����
%��³ŵ�������ﰲŵ�����ġ�������ѧ����ģ�滮����ơ���ز���
%����ο���ttps://www.jianshu.com/p/6d04539f1cfe
%% �ж������Ƿ���ȷ
[row,col]=size(dhtable);
if col~=4
    error('����DH���������ϱ�׼')
end
%% ���÷��ű���
n=row; %�ؽ�������
syms gc real;%�����������ٶ�
for i=1:n
    eval(['syms ','q',num2str(i),' real;']);            %���ùؽڱ���
    eval(['syms ','dq',num2str(i),' real;']);           %���ùؽ��ٶ�
    eval(['syms ','ddq',num2str(i),' real;']);          %���ùؽڼ��ٶ�
    eval(['syms ','m',num2str(i),' real;']);            %������������
    %ע��ո񣡣������������������ʱע��ո񣬱���������Ҳ����ã����Լ��������Ŵ���
    eval(['syms',' xc',num2str(i),' yc',num2str(i),' zc',num2str(i),' real;']);%���ùؽ��������ľ�ؽ���������ϵλ��;
    eval(['syms',' Ix',num2str(i),' Iy',num2str(i),' Iz',num2str(i),' real;']);%���ùؽ����˵������Ծ� 
    eval(['syms',' Ixy',num2str(i),' Ixz',num2str(i),' Iyz',num2str(i),' real;']);
    mass(i)=eval(['m',num2str(i),';']);                 %�����������У�����ѭ��ʱ����
    r_cen{i}=eval(['[ xc',num2str(i),';yc',num2str(i),';zc',num2str(i),'];']);%������������λ������������Ԫ������
    inertia{i}=eval(['[Ix',num2str(i),',Ixy',num2str(i),',Ixz',num2str(i),';',...
        'Ixy',num2str(i),',Iy',num2str(i),',Iyz',num2str(i),';',...
        'Ixz',num2str(i),',Iyz',num2str(i),',Iz',num2str(i),'];']);           %����ת�������������
end

%% ���ؽ�0��i�ı任����
q=sym([]);
for i=1:n
    eval(['q(i)=','q',num2str(i),'+dhtable(i,4)',';']); %��qi�͹ؽ�offset�ĺ���Ϊ�ؽڽ�
    if i==1    %0-1�ı任ʱ
        T_temp=dh_std_sy(dhtable(i,1),dhtable(i,2),dhtable(i,3),q(i));%���ɱ任����
        T{i}=simplify(T_temp);                                        %�����
    else       % 0-i�ı任ʱ��Ҫ���е������
        T_temp=dh_std_sy(dhtable(i,1),dhtable(i,2),dhtable(i,3),q(i));
        T_temp=T{i-1}*T_temp;                                          %��������
        T{i}=simplify(T_temp);
    end
end
%% ���α���Ծ���
J={};
for i=1:n
    I=par_axis(inertia{i},r_cen{i},mass(i));           %�ؽ�ԭ�㴦���Ծ���
    J{i}=JMatrix(I,r_cen{i},mass(i));                  %����Ϊ���Ծ���
end
%% ���D(q)
for i=1:n
    for j=i:n %�����ǶԳƾ��󣬿���ֻ�����Ͻǲ���
        d=0;%ÿ�����ʱ����0
        %���D(i,j)
        for k=max(i,j):n 
            d=d+trace(eval(['diff(T{k},q',num2str(i),')'])*J{k}...
                *eval(['diff(transpose(T{k}),q',num2str(j),')']));
        end
        D(i,j)=simplify(d);
        D(j,i)=D(i,j);
    end
end
%% ���H(q,dq)
for i = 1:n
    for j = 1:n
        h = 0;
        %���H
        for k = 1:n
            h=h+1/2*(eval(['diff(D(i,j),q',num2str(k),')'])+...
                eval(['diff(D(i,k),q',num2str(j),')'])-...
                eval(['diff(D(j,k),q',num2str(i),')']))*eval(['dq',num2str(k)]);
        end
       H(i,j) = simplify(h);
    end
end
%% ���G(q)
g_vec=[0,0,-gc,0]';
for i = 1:n
    g = 0;
    for j = i:n
        g=g-mass(j)*g_vec'*eval(['diff(T{j},q',num2str(i),')'])*[r_cen{j};1];
    end
    G(i) = simplify(g);
end
G = G(:);

%% ���fv
if nargout>=4
    for i=1:n
      eval(['syms',' Fv',num2str(i),' real;']);    %����Fv
      fv(i)=eval(['Fv',num2str(i),'*dq',num2str(i)]);
    end
    fv=fv(:);
end
%% ���fc
if nargout==5
    for i=1:n
        eval(['syms',' Fc',num2str(i),' real;']);   %����Fc
        fc(i)=eval(['sign(dq',num2str(i),');']);
    end
    fc=fc(:);
end
end



%% ��Ҫ���õĺ���
function T = dh_std_sy(alpha,a,d,theta)
%�˺������ñ�׼DH�������������˵ı任��ϵ
% Ϊ��������
%����Ϊ���˲���
%���Ϊ���ž���
T = [cos(theta),   -sin(theta)*cos(alpha),  sin(alpha)*sin(theta),  a*cos(theta);
    sin(theta),     cos(theta)*cos(alpha), -sin(alpha)*cos(theta),  a*sin(theta);
    0,                         sin(alpha),             cos(alpha),             d;
    0,                                  0,                      0,            1];
end

function I=par_axis(I0,r,m)
%�˺���Ϊƽ���ᶨ���ʵ��
%����I0Ϊ��ʼ����rΪ��ʼ��ָ�������,mΪ��������
%���Ϊ�յ㴦�Ĺ��Ծ���
r=r(:);   %ȷ��Ϊ������
if numrows(I0)~=3 && numcols(I0)~=3 && length(r)~=3
    error('��������ά������')
end
I=I0+m*(r'*r*eye(3)-r*r');  %ƽ���ᶨ��
end

function J = JMatrix(I,r,m)
%�˺�����������α���Ծ����䶨��ο������׵�����������ѧ����ģ�������Ӿ���7.2.2�����ݡ�
%����IΪ���Ծ���rΪ���������ԭ��λ��������mΪ����
%���JΪα���Ծ���
if numrows(I)~=3 && numcols(I)~=3 && length(r)~=3
    error('��������ά������')
end
r=r(:);
k=(I(1,1)+I(2,2)+I(3,3))/2;
I_temp=-I+diag([k,k,k]);
J=[I_temp m*r;
    m*r' m];
end