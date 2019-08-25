function [q,qd,qdd]=tracj_t(q0,qf,tf,T,qm,num)
%利用梯形函数拟合，起始角度为q0,终点角度为qf，tf为时间
%T用于声明qm的类型，包含‘V’(速度)和'A'(加速度两个选项)
%num为时间点数量，不输入时默认为100；
%qdd为加速阶段的加速度绝对值
%qd为匀速阶段的速度绝对值
%输出为角度序列q，速度序列qd，加速度序列qdd

%% 判断输入值是否符合要求
if length(q0)~=length(qf)
    error('起始和终点角度维数不同')
end
if length(qm)~=length(q0)
    error('加速度和角度维数不同')
end
if nargin==5
    num=100;
end
%% 数据处理
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
                    error('输入速度不符合要求')
                end
            end
            tc=tf-(qf-q0)./qm;
            qd_temp=qm;
            qdd_temp=qm./tc;
        case 'A'
            qdd_temp=qm;
            for i=1:length(q0)
                if (abs(qdd_temp(i)))<(4*abs((qf(i)-q0(i)))/tf^2)
                    error('输入加速度不符合要求')
                end
            end
            tc=-1/2*sqrt((-4*(qf-q0)+(tf^2*qdd_temp))./qdd_temp)+tf/2;
            qd_temp=qdd_temp.*tc;
    end
        
 %% 求解输出

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