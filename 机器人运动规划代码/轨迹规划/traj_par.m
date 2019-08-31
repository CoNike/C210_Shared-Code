function [q,qd,qdd]=traj_par(q0,td,qm)
%�����߹��ɵ����Բ�ֵ������Ϊ���ɵ�Ƕ�����q0(MxN)��ʱ������t
%ʱ��Ϊ���ڽǶ�ֵ֮��Ĺ���ʱ��,MΪ������NΪ�����˹ؽ���
%���Ϊ�Ƕ�����q�����ٶ�����qd���Ǽ��ٶ�����qdd
%qmΪÿ���ؽڵĽǶ�
%% �ж����������Ƿ����Ҫ��
[q0row,q0col]=size(q0);
if (q0row-1)~=length(td)
    error('����ʱ�䳤���뾭�ɵ㲻��Ӧ')
end
if q0col~=length(qm)
    error('����Ƕ�����ٶ�ά������Ӧ')
end
qm=abs(qm(:));
qd1=zeros(length(td),q0col);
%�ж�������ٶ��Ƿ����Ҫ��
for i=1:length(td)
    qd1(i,:)=(q0(i+1,:)-q0(i,:))/td(i);
    for j=1:q0col
        if abs(qd1(i,j))>2*td*qm(j)
            error('������ٶȲ�����Ҫ��')
        end
    end
end

%% ������׶��ٶ�
qd1=[zeros(1,q0col);
    qd1;
    zeros(1,q0col);];

%% �������ʱ�������ʱ��
t1=zeros(q0row,q0col); %����ʱ��
t2=zeros(q0row-1,q0col);%����ʱ��
for i=1:q0row
    for j=1:q0col
        t1(i,j)=(qd1(i+1,j)-qd1(i,j))/(sign(qd1(i+1,j)-qd1(i,j))*qm(j));
        if i<q0row
        t2(i,j)=td(i)-t1(i,j)/2-t1(i+1,j)/2;
        end
    end 
end
%% ������ʱ��
t_sum=sum(td)+max(t1(1,:))/2+max(t1(q0row,:));

%% ������
%׼������
t=0:t_sum/500:t_sum;
q=zeros(length(t),q0col);
qd=zeros(length(t),q0col);
qdd=zeros(length(t),q0col);
qtemp=zeros(20,3);


for k=1:length(t)
    for j=1:q0col
        %��ʼ�׶�
        if t(k)<(max(t1(1,:))/2-t1(1,j)/2)
            q(k,j)=q(1,j);
            qd(k,j)=0;
            qdd(k,j)=0;
            qtemp(1,:)=[q(1,j) 0 0 ];
        %��һ�����ٽ׶�
        elseif (t(k)>=(max(t1(1,:))/2-t1(1,j)/2)) && (t(k)<(max(t1(1,:))/2+t1(1,j)/2))
            ts=t(k)-(max(t1(1,:))/2-t1(1,j)/2);
            qdd_temp=sign(q0(2,j)-q0(1,j))*qm(j);
            qdd(k,j)=qdd_temp;
            qd(k,j)=(ts)*qdd_temp+qtemp(1,2);
            q(k,j)=1/2*(ts)^2*qdd_temp+qtemp(1,3);
            qtemp(2,1)=qdd_temp;
            qtemp(2,2)=(t1(1,j))*qdd_temp+qtemp(1,2);
            qtemp(2,3)=1/2*(t1(1,j))^2*qdd_temp+qtemp(1,3);
        %��һ�����ٽ׶�
        elseif (t(k)>=(max(t1(1,:))/2+t1(1,j)/2)) && (t(k)<(max(t1(1,:))/2+sum((td(1))-t1(1+1,j)/2)))
            ts=t(k)-(max(t1(1,:))/2+t1(1,j)/2);
            qdd(k,j)=0;
            qd(k,j)=qd1(2,j);
            q(k,j)=qtemp(2,3)+qtemp(2,2)*(ts);
            qtemp(3,1)=0;
            qtemp(3,2)=qd1(2,j);
            qtemp(3,3)=qtemp(2,3)+qtemp(3,2)*(td(1)-t1(1,j)/2-t1(1+1,j)/2);
        %���һ�����ٽ׶�
        elseif (t(k)>=(max(t1(1,:))/2+sum(td)-t1(q0row,j)/2)) && (t(k)<(max(t1(1,:))/2+sum(td)+t1(q0row,j)/2))
            ts=t(k)-(max(t1(1,:))/2+sum(td)-t1(q0row,j)/2);
            qdd_temp=sign(qd1(q0row+1,j)-qd1(q0row,j))*qm(j);
            qdd(k,j)=qdd_temp;
            qd(k,j)=(ts)*qdd_temp+qtemp(2*(q0row)-1,2);
            q(k,j)=1/2*(ts)^2*qdd_temp+qtemp(2*(q0row)-1,3)+qtemp(2*(q0row)-1,2)*ts;
        %��β�׶�
        elseif (t(k)>(max(t1(1,:))/2+t1(1,j)/2+sum(td))) && (t(k)<t_sum)
            qdd(k,j)=0;
            qd(k,j)=0;
            q(k,j)=q0(q0row,j);
        %�м�׶�
        else            
            for i=2:q0row-1
                %���ٽ׶�
                if (t(k)>=(max(t1(1,:))/2-t1(i,j)/2+sum(td(1:i-1)))) && (t(k)<(max(t1(1,:))/2+t1(i,j)/2+sum(td(1:i-1))))
                    ts=t(k)-(max(t1(1,:))/2-t1(i,j)/2+sum(td(1:i-1)));
                    qdd_temp=sign(qd1(i+1,j)-qd1(i,j))*qm(j);
                    qdd(k,j)=qdd_temp;
                    qd(k,j)=ts*qdd_temp+qtemp(2*(i)-1,2);
                    q(k,j)=1/2*(ts)^2*qdd_temp+qtemp(2*i-1,3)+qtemp(2*(i)-1,2)*ts;
                    qtemp(2*(i),1)=qdd_temp;
                    qtemp(2*(i),2)=(t1(i,j))*qdd_temp+qtemp(2*(i)-1,2);
                    qtemp(2*(i),3)=1/2*(t1(i,j))^2*qdd_temp+qtemp(2*(i)-1,3)+(t1(i,j))*qtemp(2*(i)-1,2);
                %���ٽ׶�
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