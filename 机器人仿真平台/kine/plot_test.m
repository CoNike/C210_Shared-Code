% plot_test
% robot=p560xml;
% q=[0 1 1 0 0 0];
% n=robot.n;
% if length(q)~=n
%     error('输入角度维数不对')
% end
%获取机器人的图形参数用于绘图
% faces=robot.faces;
% points=robot.points;
% %获取基座的图形数据
% F0=faces{1};
% V0=points{1};
% twist=robot.twist;
% 
% L0=patch('Faces',F0,'Vertices',V0);
% set(L0,'FaceColor',[0.5 0.5 0.5],'EdgeColor','none')
% 
% hold on;
% 
% T=SE3(0,0,0);
% for i=1:n
%     F=faces{i+1};
%     V_o=points{i+1};
%     tw_vec=twist(i,:);
%     tw=Twist(tw_vec);
%     T=T*SE3(tw.T(q(i)));
%     V_t=T*V_o';
%     L=patch('Faces',F,'Vertices',V_t');
%     set(L,'FaceColor',[0.5 0.5 0.5],'EdgeColor','none')
% end


% 测试原始图形文件
%puma560原始图形文件有误

% hold on;

% [p0,f0]=stlRead('robot/epsonG5651S/GS651S - base_1.STL');
% L0=patch('Faces',f0,'Vertices',p0);
% set(L0,'FaceColor',[0.5 0.5 0.5],'EdgeColor','none');
% [p0,f0]=stlRead('robot/epsonG5651S/GS651S - joint1_1.STL');
% L0=patch('Faces',f0,'Vertices',p0);
% set(L0,'FaceColor',[0.5 0.5 0.5],'EdgeColor','none');
% [p0,f0]=stlRead('robot/epsonG5651S/GS651S - joint2_1.STL');
% L0=patch('Faces',f0,'Vertices',p0);
% set(L0,'FaceColor',[0.5 0.5 0.5],'EdgeColor','none');
% set(L0,'FaceColor',[0.5 0.5 0.5],'EdgeColor','none');
% [p0,f0]=stlRead('robot/UR5/UR5 - joint2_2.STL');
% L0=patch('Faces',f0,'Vertices',p0);
% set(L0,'FaceColor',[0.5 0.5 0.5],'EdgeColor','none');
% 
% [p1,f1]=stlRead('robot/link1.stl');
% L1=patch('Faces',f1,'Vertices',p1);
% set(L1,'FaceColor',[0.5 0 0],'EdgeColor','none');
% 
% [p2,f2]=stlRead('robot/link2.stl');
% L2=patch('Faces',f2,'Vertices',p2);
% set(L2,'FaceColor',[0 0.5 0],'EdgeColor','none');