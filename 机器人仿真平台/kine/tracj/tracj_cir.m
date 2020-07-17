function T=tracj_cir(T1,T2,T3,t)
%该函数用于在三个位姿之间生成一段圆弧轨迹
%输入T1,T2,T3为三个位姿矩阵，t为步数
%输出T为连续的序列
%考虑到计算方便姿态采用rpy然后进行圆弧插值
%求解末端位置的
p1=transl(T1);
p2=transl(T2);
p3=transl(T3);
p=circle_jtraj(p1,p2,p3,t);
%求解rpy
rpy1=tr2rpy(T1);
rpy2=tr2rpy(T2);
rpy3=tr2rpy(T3);
rpy=circle_jtraj(rpy1,rpy2,rpy3,t);
T=[rpy p];
end