function   q0=UR10Num2(TG,q0)
mdl_ur10;
n=0;
flag=1;
alpha=0.1;
lambda=0.01;
while flag==1
  T0=ur10.fkine(q0);
  T0=T0.T;
  T=TG*inv(T0);
  T1=SE3(T);
  delta=T1.todelta;
  jaco=ur10.jacob0(q0);
  temp=jaco'*inv(jaco*jaco'+lambda*eye(6))*delta;
  q0=q0+alpha*temp;
  n=n+1;
  dis(n)=norm(delta);
  if (abs(dis(n))<10^-6)||(n>2000)
      flag=0;
  end

end
%  plot(dis)
end