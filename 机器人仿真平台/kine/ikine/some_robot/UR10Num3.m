function   [q0,n,dis]=UR10Num3(TG,q0)
mdl_ur10;
n=2;
dis=[0 0];
flag=1;
alpha=1.5;
lambda=0.01;
while flag==1
  T0=ur10.fkine(q0);
  T0=T0.T;
  T=TG*inv(T0);
  T1=SE3(T);
  delta=T1.todelta;
  jaco=ur10.jacob0(q0);
  det_j=abs((det(jaco)));
  dis(n)=norm(delta);
%   if dis(n)<10^-2
%       alpha=1;
%   end
  temp=jaco'*inv(jaco*jaco'+lambda*eye(6))*delta;
  q0=q0+alpha*temp;
  if (abs(dis(n)-dis(n-1))<10^-8)||(n>2000)
      flag=0;
  end
  n=n+1;
end
%  plot(dis)
end