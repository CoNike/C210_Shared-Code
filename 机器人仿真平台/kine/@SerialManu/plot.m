%机器人三维模型绘制函数，输入
function h=plot(robot,q)
if ~isa(robot,'SerialManu')
    error('输入模型不对')
end
n=robot.n;
if length(q)~=n
    error('输入角度维数不对')
end
%获取机器人的图形参数用于绘图
faces=robot.faces;
points=robot.points;
%获取基座的图形数据
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
        num=length(F);%确定该关节绘图数量
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