function Tq=fkine_screw(twist,T0,q)
%利用旋量twist，起始位置T0和关节角度q
%输入twist为一个6xN的矩阵，N为关节数，前三行轴线旋转的方向，
%后三行为轴线上一点；
%输入T0为末端执行器的起始位姿矩阵，q为关节角度，单位为弧度
%输出Tq为机器人旋转q后末端执行器的位姿
%% 判断输入是否符合要求
[twrow,twcol]=size(twist);
if (twrow~=6)||(twcol~=length(q))
    error('输入的参数不规范')
end
%% 利用旋量求变换矩阵
%此处没必要生成元胞数组
T=eye(4);
for i=1:twcol
    tw=Twist('R',twist(1:3,i),twist(4:6,i));   %构造旋量
    T=T*tw.T(q(i));                            %生成变换矩阵
end
% tw=cell(1,twcol);
% T=cell(1,twcol);
% for i=1:twcol
%     tw{i}=Twist('R',twist(1:3,i),twist(4:6,i));
%     T{i}=tw{i}.T(q(i));
%     if i>1
%         T{i}=T{i-1}*T{i};
%     end
% end
%% 求Tq
if isa(T0,'SE3')
    T0=T0.T;
end
% Tq=T{twcol}*T0;
Tq=T*T0;
end