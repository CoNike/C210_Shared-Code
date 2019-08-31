function [q,qd,qdd]=traj_par(q0,td,qm)
%抛物线过渡的线性插值，输入为经由点角度序列q0(MxN)和时间序列t
%时间为相邻角度值之间的过渡时间,M为点数，N为机器人关节数
%输出为角度序列q，角速度序列qd，角加速度序列qdd
%qm为每个关节的角度
%% 判断输入数据是否符合要求
[q0row,q0col]=size(q0);
if (q0row-1)~=length(td)
    error('输入时间长度与经由点不对应')
end
if q0col~=length(qm)
    error('输入角度与加速度维数不对应')
end
qm=abs(qm(:));
qd1=zeros(length(td),q0col);
%判断输入加速度是否符合要求
for i=1:length(td)
    qd1(i,:)=(q0(i+1,:)-q0(i,:))/td(i);
    for j=1:q0col
        if abs(qd1(i,j))>2*td*qm(j)
            error('输入加速度不符合要求')
        end
    end
end

%% 计算各阶段速度
qd1=[zeros(1,q0col);
    qd1;
    zeros(1,q0col);];

%% 计算变速时间和匀速时间
t1=zeros(q0row,q0col); %变速时间
t2=zeros(q0row-1,q0col);%匀速时间
for i=1:q0row
    for j=1:q0col
        t1(i,j)=(qd1(i+1,j)-qd1(i,j))/(sign(qd1(i+1,j)-qd1(i,j))*qm(j));
        if i<q0row
        t2(i,j)=td(i)-t1(i,j)/2-t1(i+1,j)/2;
        end
    end 
end
%% 计算总时间
t_sum=sum(td)+max(t1(1,:))/2+max(t1(q0row,:));

%% 求解输出
%准备数据
t=0:t_sum/500:t_sum;
q=zeros(length(t),q0col);
qd=zeros(length(t),q0col);
qdd=zeros(length(t),q0col);
qtemp=zeros(20,3);


for k=1:length(t)
    for j=1:q0col
        %开始阶段
        if t(k)<(max(t1(1,:))/2-t1(1,j)/2)
            q(k,j)=q(1,j);
            qd(k,j)=0;
            qdd(k,j)=0;
            qtemp(1,:)=[q(1,j) 0 0 ];
        %第一个变速阶段
        elseif (t(k)>=(max(t1(1,:))/2-t1(1,j)/2)) && (t(k)<(max(t1(1,:))/2+t1(1,j)/2))
            ts=t(k)-(max(t1(1,:))/2-t1(1,j)/2);
            qdd_temp=sign(q0(2,j)-q0(1,j))*qm(j);
            qdd(k,j)=qdd_temp;
            qd(k,j)=(ts)*qdd_temp+qtemp(1,2);
            q(k,j)=1/2*(ts)^2*qdd_temp+qtemp(1,3);
            qtemp(2,1)=qdd_temp;
            qtemp(2,2)=(t1(1,j))*qdd_temp+qtemp(1,2);
            qtemp(2,3)=1/2*(t1(1,j))^2*qdd_temp+qtemp(1,3);
        %第一个匀速阶段
        elseif (t(k)>=(max(t1(1,:))/2+t1(1,j)/2)) && (t(k)<(max(t1(1,:))/2+sum((td(1))-t1(1+1,j)/2)))
            ts=t(k)-(max(t1(1,:))/2+t1(1,j)/2);
            qdd(k,j)=0;
            qd(k,j)=qd1(2,j);
            q(k,j)=qtemp(2,3)+qtemp(2,2)*(ts);
            qtemp(3,1)=0;
            qtemp(3,2)=qd1(2,j);
            qtemp(3,3)=qtemp(2,3)+qtemp(3,2)*(td(1)-t1(1,j)/2-t1(1+1,j)/2);
        %最后一个变速阶段
        elseif (t(k)>=(max(t1(1,:))/2+sum(td)-t1(q0row,j)/2)) && (t(k)<(max(t1(1,:))/2+sum(td)+t1(q0row,j)/2))
            ts=t(k)-(max(t1(1,:))/2+sum(td)-t1(q0row,j)/2);
            qdd_temp=sign(qd1(q0row+1,j)-qd1(q0row,j))*qm(j);
            qdd(k,j)=qdd_temp;
            qd(k,j)=(ts)*qdd_temp+qtemp(2*(q0row)-1,2);
            q(k,j)=1/2*(ts)^2*qdd_temp+qtemp(2*(q0row)-1,3)+qtemp(2*(q0row)-1,2)*ts;
        %结尾阶段
        elseif (t(k)>(max(t1(1,:))/2+t1(1,j)/2+sum(td))) && (t(k)<t_sum)
            qdd(k,j)=0;
            qd(k,j)=0;
            q(k,j)=q0(q0row,j);
        %中间阶段
        else            
            for i=2:q0row-1
                %变速阶段
                if (t(k)>=(max(t1(1,:))/2-t1(i,j)/2+sum(td(1:i-1)))) && (t(k)<(max(t1(1,:))/2+t1(i,j)/2+sum(td(1:i-1))))
                    ts=t(k)-(max(t1(1,:))/2-t1(i,j)/2+sum(td(1:i-1)));
                    qdd_temp=sign(qd1(i+1,j)-qd1(i,j))*qm(j);
                    qdd(k,j)=qdd_temp;
                    qd(k,j)=ts*qdd_temp+qtemp(2*(i)-1,2);
                    q(k,j)=1/2*(ts)^2*qdd_temp+qtemp(2*i-1,3)+qtemp(2*(i)-1,2)*ts;
                    qtemp(2*(i),1)=qdd_temp;
                    qtemp(2*(i),2)=(t1(i,j))*qdd_temp+qtemp(2*(i)-1,2);
                    qtemp(2*(i),3)=1/2*(t1(i,j))^2*qdd_temp+qtemp(2*(i)-1,3)+(t1(i,j))*qtemp(2*(i)-1,2);
                %匀速阶段
                elseif (t(k)>=(max(t1(1,:))/2+t1(i,j)/2+sum(td(1:i-1)))) && (t(k)<((max(t1(1,:))/2+sum(td(1:i))-t1(i+1,j)/2)))
                    ts=t(k)-(max(t1(1,:))/2+t1(i,j)/2+sum(td(1:i-1)));
                    qdd(k,j)=0;
                    qd(k,j)=qd1(i+1,j);
                    q(k,j)=qtemp(2*(i),3)+qtemp(2*(i),2)*ts;
                    qtemp(2*i+1,1)=0;
                    qtemp(2*i+1,2)=qtemp(2*i,2);
                    qtemp(2*i+1,3)=qtemp(2*i,3)+qtemp(2*i+1,2)*(td(i)-t1(i,j)/2-t1(i+1,j)/2);
                end
            end
        end
    end
end       
end