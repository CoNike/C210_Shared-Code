function J=jaco(robot,q)
%求解机器人雅克比矩阵
%输入robot为机器人结构体（SerialLink），q为机器人关节角度
%输出J为雅克比矩阵
q=q(:)';
n=robot.n;
if length(q)~=n
    error('输入数据有误')
end
%获取机器人参数
alpha=robot.alpha;
a=robot.a;
d=robot.d;
offset=robot.offset;
dh=[alpha' a' d' offset'];
J=jacobin0(dh,q);
if isa(q,'sym')
    J=simplify(J);
end
end