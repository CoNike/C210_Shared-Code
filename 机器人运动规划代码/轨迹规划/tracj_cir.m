function T=tracj_cir(T1,T2,T3,t)
%�ú�������������λ��֮������һ��Բ���켣
%����T1,T2,T3Ϊ����λ�˾���tΪ����
%���TΪ����������
%���ǵ����㷽����̬����rpyȻ�����Բ����ֵ
%���ĩ��λ�õ�
p1=transl(T1);
p2=transl(T2);
p3=transl(T3);
p=circle_jtraj(p1,p2,p3,t);
%���rpy
rpy1=tr2rpy(T1);
rpy2=tr2rpy(T2);
rpy3=tr2rpy(T3);
rpy=circle_jtraj(rpy1,rpy2,rpy3,t);
T=[rpy p];
end