%��������άģ�ͻ��ƺ���������
function h=plot(robot,q)
if ~isa(robot,'SerialManu')
    error('����ģ�Ͳ���')
end
n=robot.n;
if length(q)~=n
    error('����Ƕ�ά������')
end
%��ȡ�����˵�ͼ�β������ڻ�ͼ
faces=robot.faces;
points=robot.points;
%��ȡ������ͼ������
F0=faces{1};
V0=points{1};
twist=robot.twist;

L0=patch('Faces',F0,'Vertices',V0);
set(L0,'FaceColor',[0.5 0.5 0.5],'EdgeColor','none')
h=L0;
hold on;

T=SE3(0,0,0);
for i=1:n
    F=faces{i+1};
    V_o=points{i+1};
    if isa(F,'cell')
        num=length(F);%ȷ���ùؽڻ�ͼ����
    else
        num=1;
    end
    if num==1
        tw_vec=twist(i,:);
        tw=Twist(tw_vec);
        T=T*SE3(tw.T(q(i)));
        if ~isempty(V_o)
            V_t=T*V_o';
            L=patch('Faces',F,'Vertices',V_t');
            set(L,'FaceColor',[0.5 0.5 0.5],'EdgeColor','none')
            h=[h,L];
        end
    else
        tw_vec=twist(i,:);
        tw=Twist(tw_vec);
        T=T*SE3(tw.T(q(i)));
        for j=1:num
            F_muti=F{j};
            V_muti=V_o{j};
            V_t=T*V_muti';
            L=patch('Faces',F_muti,'Vertices',V_t');
            set(L,'FaceColor',[0.5 0.5 0.5],'EdgeColor','none')
            h=[h,L];
        end
    end 
end
end