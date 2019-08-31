function   [q0,n,dis]=UR10Num1(TG,q0)
mdl_ur10;
n=2;
dis=[1 0.8];
flag=1;
alpha=1;
while flag==1
  T0=ur10.fkine(q0);
  T0=T0.T;
  T=TG*inv(T0);
  T1=SE3(T);
  delta=T1.todelta;
  jaco=ur10.jacob0(q0);
  det_j=abs((det(jaco)));
  temp=inv(jaco)*delta;
  q0=q0+alpha*temp;
  n=n+1;
  dis(n)=norm(delta);
  if (abs(dis(n))<10^-6)||(n>2000)
      flag=0;
  end
end
%  plot(dis)
end