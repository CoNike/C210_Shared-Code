%有负载时，peter工具包计算似乎不对。
%puma560,test,dyn  
mdl_puma560
t1=dyn_lagr_p560(qn,qn,qz)
t2=rne_dh(p560,qn,qn,qz)
t3=idyna_nr_dh(p560,qn,qn,qz)


t11=dyn_lagr_p560(qn,qn,qz,[1 1 1 1  1  1])
t22=rne_dh(p560,qn,qn,qz,p560.gravity,[1 1 1 1 1 1])
t33=idyna_nr_dh(p560,qn,qn,qz,p560.gravity,[1 1 1 1 1 1])
m1=t11-t1;
m2=t22-t2;
m3=t33-t3;