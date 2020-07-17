function J=jaco(robot,q)
%���������ſ˱Ⱦ���
%����robotΪ�����˽ṹ�壨SerialLink����qΪ�����˹ؽڽǶ�
%���JΪ�ſ˱Ⱦ���
q=q(:)';
n=robot.n;
if length(q)~=n
    error('������������')
end
%��ȡ�����˲���
alpha=robot.alpha;
a=robot.a;
d=robot.d;
offset=robot.offset;
dh=[alpha' a' d' offset'];
J=jacobin0(dh,q);
if isa(q,'sym')
    J=simplify(J);
end
end