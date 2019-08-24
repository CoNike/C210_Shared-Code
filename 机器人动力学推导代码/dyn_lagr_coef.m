function [D,H,G,fv]=dyn_lagr_coef(robot,q,qd,gravity)
%�˺������������ն���ѧ�����������˶���ѧ,
%Ŀǰֻ������ת�ؽ�,�����ǵ����Ӱ�졣
%[D,H,G,fv,fc]=dyn_lagr_coef(robot,q,qd,qdd,gravity,f_end)
%����robotΪ������ģ�ͣ�
%qΪ�����˹ؽڱ�����
%qdΪ�����˹ؽڽ��ٶȣ�
%gravityΪ��������������
%f_endΪ������ĩ��ִ����������
%���DΪ���ٶ����ϵ�����󣬱������Ǽ��ٶȶԹؽ�����Ӱ�죻
%���HΪ�ٶ����ϵ�����󣬱��������ٶȶԸ��ؽ�����Ӱ�죻
%���GΪ������
%fv��fcΪѡ�������
%���fvΪ���ؽڵ�ճ��Ħ����
%���fcΪ���ؽڵĿ���Ħ����
%�ο�����Ϊ�����׵����ġ�������ѧ����ģ���������Ӿ�����
%��³ŵ�������ﰲŵ�����ġ�������ѧ����ģ�滮����ơ���ز���
%����ο���ttps://www.jianshu.com/p/6d04539f1cfe
n=robot.n;  %�ؽ���
if length(q)~=n && length(qd)~=qd
    error('��������ά������')
end
gc=robot.gravity;%����
if nargin==4
    gc=gravity;
end
%��ȡDH������������DH������
for i=1:n
    eval(['syms ','at',num2str(i),' real;']);
    eval(['syms ','d',num2str(i),' real;']);
    a(i)=eval(['at',num2str(i)]);
    d(i)=eval(['d',num2str(i)]);
end
alpha=robot.alpha;
offset=robot.offset;
dhtable=[alpha' a' d' offset'];
%���ɷ��ž���
[D,H,G]=dyn_lagr_sym(dhtable);
%�Է��Ž��и�ֵ
for i=1:n
  link=robot.links(i);
  eval(['at',num2str(i),'=','link.a',';']);
  eval(['d',num2str(i),'=','link.d',';']);
  eval(['q',num2str(i),'=','q(i)',';']);
  eval(['dq',num2str(i),'=','qd(i)',';']);
  eval(['m',num2str(i),'=','link.m',';']);
  eval(['xc',num2str(i),'=','link.r(1)',';']);
  eval(['yc',num2str(i),'=','link.r(2)',';']);
  eval(['zc',num2str(i),'=','link.r(3)',';']);
  eval(['Ix',num2str(i),'=','link.I(1,1)',';']);
  eval(['Iy',num2str(i),'=','link.I(2,2)',';']);
  eval(['Iz',num2str(i),'=','link.I(3,3)',';']);
  eval(['Ixy',num2str(i),'=','link.I(1,2)',';']);
  eval(['Ixz',num2str(i),'=','link.I(1,3)',';']);
  eval(['Iyz',num2str(i),'=','link.I(2,3)',';']);
  fv(i)=link.friction(qd(i));
end
gc=gc(3);
%����ϵ������
D=subs(D);
H=subs(H);
G=subs(G);
fv=fv(:);
end