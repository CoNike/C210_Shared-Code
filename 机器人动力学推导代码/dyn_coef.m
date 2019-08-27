function [D,H,G,fv,fc]=dyn_coef(robot,q,qd)
%�˺������������ն���ѧ�����������˶���ѧ����Ҫ���ڲ����������ն���ѧ�㷨����ȷ��
%���ʹ����ֵ�����Ŀǰֻ������ת�ؽ�,�����ǵ����Ӱ��
% [D,H,G,fv,fc]=dyn_coef(robot,q,qd)
%robotΪ������ģ�ͣ������л������˶�ѧ�Ͷ���ѧ��ز���
%qΪ�����˹ؽڽǶȣ�qdΪ�����˹ؽڽ��ٶ�
%���DΪ���ٶ����ϵ�����󣬱������Ǽ��ٶȶԹؽ�����Ӱ�죻
%���HΪ�ٶ����ϵ�����󣬱��������ٶȶԸ��ؽ�����Ӱ�죻
%���GΪ������
%fv��fcΪѡ�������д�ȷ���ͱ궨
%���fvΪ���ؽڵ�ճ��Ħ����
%���fcΪ���ؽڵĿ���Ħ����
%�ο�����Ϊ�����׵����ġ�������ѧ����ģ���������Ӿ�����
%��³ŵ�������ﰲŵ�����ġ�������ѧ����ģ�滮����ơ���ز���
%����ο���ttps://www.jianshu.com/p/6d04539f1cfe
%% �ж������Ƿ���ȷ
n=robot.n;%��ȡ������Ŀ
q=q(:);
qd=qd(:);
if length(q)~=n||length(qd)~=n
    error('error data!!')
end
%% ���ò���
grav=robot.gravity;%�����������ٶ�
 %��ȡ���˲���
for i=1:n
    eval(['syms ','q',num2str(i),' real;']);            %���ùؽڱ���
    eval(['syms ','dq',num2str(i),' real;']);          %���ùؽ��ٶ�
    qk(i)=eval(['q',num2str(i)]);
    link=robot.links(i);
    mass(i)=link.m;
    inertia{i}=link.I;
    r_cen{i}=link.r;
end
a=robot.a;
d=robot.d;
alpha=robot.alpha;
offset=robot.offset+qk;%qk�ڴ˴�����ܹ���������
dhtable=[alpha' a' d' offset'];
%% ���ؽ�0��i�ı任����
%�˴����õ��±�׼DHģ��
for i=1:n
    if i==1    %0-1�ı任ʱ
        T_temp=dh_std_sy(dhtable(i,1),dhtable(i,2),dhtable(i,3),dhtable(i,4));%���ɱ任����
        T{i}=simplify(T_temp);                                        %�����
    else       % 0-i�ı任ʱ��Ҫ���е������
        T_temp=dh_std_sy(dhtable(i,1),dhtable(i,2),dhtable(i,3),dhtable(i,4));
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
    for j=i:n %�����ǶԳƾ��󣬿���ֻ�������ǲ���
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
g_vec=[-grav;0];
for i = 1:n
    g = 0;
    for j = i:n
        g=g-mass(j)*g_vec'*eval(['diff(T{j},q',num2str(i),')'])*[r_cen{j} 1]';
    end
    G(i) = simplify(g);
end
G = G(:);
%% ��ֵ���
for i=1:n
    eval(['q',num2str(i),' =q(i);']);            %���ùؽڱ���
    eval(['dq',num2str(i),'=qd(i);']);          %���ùؽ��ٶ�
end
D=subs(D);
H=subs(H);
G=subs(G);
%% ���fv
if nargout>=4
    for i=1:n
        link=robot.links(i);
      fv(i)=link.friction(qd);
    end
    fv=fv(:);
end
%% ���fc
%Ŀǰ�����˻�δ������ϵ��
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