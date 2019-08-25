function [q,qd,qdd]=tracj_t(q0,qf,tf,T,qm,num)
%�������κ�����ϣ���ʼ�Ƕ�Ϊq0,�յ�Ƕ�Ϊqf��tfΪʱ��
%T��������qm�����ͣ�������V��(�ٶ�)��'A'(���ٶ�����ѡ��)
%numΪʱ���������������ʱĬ��Ϊ100��
%qddΪ���ٽ׶εļ��ٶȾ���ֵ
%qdΪ���ٽ׶ε��ٶȾ���ֵ
%���Ϊ�Ƕ�����q���ٶ�����qd�����ٶ�����qdd

%% �ж�����ֵ�Ƿ����Ҫ��
if length(q0)~=length(qf)
    error('��ʼ���յ�Ƕ�ά����ͬ')
end
if length(qm)~=length(q0)
    error('���ٶȺͽǶ�ά����ͬ')
end
if nargin==5
    num=100;
end
%% ���ݴ���
q0=q0(:);
qf=qf(:);
tf=abs(tf);
qdir=sign(qf-q0);
qm=abs(qm(:)).*qdir;

if ischar(T)
    switch upper(T)
        case 'V'
            for i=1:length(q0)
                if ((abs(qm(i)))<=(abs(qf(i)-q0(i))/tf))||((abs(qm(i)))>2*(abs(qf(i)-q0(i))/tf))
                    error('�����ٶȲ�����Ҫ��')
                end
            end
            tc=tf-(qf-q0)./qm;
            qd_temp=qm;
            qdd_temp=qm./tc;
        case 'A'
            qdd_temp=qm;
            for i=1:length(q0)
                if (abs(qdd_temp(i)))<(4*abs((qf(i)-q0(i)))/tf^2)
                    error('������ٶȲ�����Ҫ��')
                end
            end
            tc=-1/2*sqrt((-4*(qf-q0)+(tf^2*qdd_temp))./qdd_temp)+tf/2;
            qd_temp=qdd_temp.*tc;
    end
        
 %% ������

  t=0:tf/(num-1):tf;
  q0=q0';
  qf=qf';
  qd_temp=qd_temp';
  qdd_temp=qdd_temp';
  q=zeros(length(t),length(q0));
  qd=zeros(length(t),length(q0));
  qdd=zeros(length(t),length(q0));
  tc=tc';
  for j=1:length(q0)
      for i=1:length(t)
          if t(i)<tc(j)
              q(i,j)=q0(j)+1/2*qdd_temp(j)*t(i)^2;
              qd(i,j)=qdd_temp(j)*t(i);
              qdd(i,j)=qdd_temp(j);
          elseif t(i)>tf-tc(j)
              q(i,j)=qf(j)-1/2*qdd_temp(j)*(t(i)-tf)^2;
              qd(i,j)=qdd_temp(j)*(tf-t(i));
              qdd(i,j)=-qdd_temp(j);
          else
              q(i,j)=q0(j)+1/2*qdd_temp(j)*(tc(j).^2)+qd_temp(j)*(t(i)-tc(j));
              qd(i,j)=qd_temp(j);
              qdd(i,j)=0;
          end
      end
  end
 end
        
end