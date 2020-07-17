function   [q0,n,dis]=UR10Num4(TG,q0)
% mdl_ur10;
n=2;
flag=1;
dis=zeros(1,30);

while flag==1
  T0=ur10.fkine(q0);
  T0=T0.T;
  T=TG*inv(T0);
  T1=SE3(T);
  delta=T1.todelta; 
  jaco=ur10.jacob0(q0);
  det_j=abs((det(jaco)));
  if  det_j>10^-4
       temp=inv(jaco)*delta;
       alpha=1;
  else
       lambda=0.01;
       temp=jaco'*inv(jaco*jaco'+lambda*eye(6))*delta;
       alpha=1.5;
  end
  q0=q0+alpha*temp;
  n=n+1
  dis(n)=norm(delta);
 if (abs(dis(n)-dis(n-1))<10^-8)||(n>2000)
      flag=0;
  end
end
%  plot(dis)
end