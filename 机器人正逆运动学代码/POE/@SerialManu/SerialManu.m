%创建基于指数积公式的串联机器人模型
%后续可能应用并联机器人和树形机器人的创建
%robot=SerialManu();创建一个空的机器人
%robot=SerialManu(robot);复制另外一个机器人，会复制相同的参数
%robot=SerialManu([jolink1 jolink2],'T0',4X4double,...(option))
%利用参数创建机器人模型
%其中[jolink1 jolink2]为JoLink对象矩阵，其具体定义可以参考JoLink类
%option:
%'T0',4X4double或SE3，机器人末端执行器的初始位姿,必须赋值
%'name',char,机器人名称（default,''）
%'comment',char,机器人相关说明（default,''）
%'gravity',3X1double,机器人重力加速度方向（default,[0 0 9.81]）
%'plot3dopt',char,机器人3D绘制的方式（default,{}）暂时还未定义
%ikineType ，char,机器人你运动学类型（default,{}）暂时还未定义

%关于指数积的内容可以参考一下文献：
%熊有伦等《机器人学：建模控制与视觉》
%李泽湘等《机器人学的几何基础》
%代码可以参考Peter Corke的RTB工具箱

%creator: Huang Zhouzhou  Time:2019/9/6
%Huazhong University of Science and Technology
classdef SerialManu < handle 
     properties
         name       %机器人名称
         comment    %机器人相关说明
         T0         %设定机器人末端相对于基坐标系的初始位姿
         gravity    %重力加速度矢量
         base       %机器人基座坐标系变换
         tool       %机器人工具坐标系变换
         plot3dopt     %3d绘制
         ikineType  %逆解类型
         faces      %读取几何文件之后的面和点
         points
     end
     events
         moved
     end
     properties (SetAccess = private)
         n
         jolinks
         T
     end
     properties (Dependent = true)
         %此处为从属属性，从属属性不能通过赋值定义
         %只能用get函数定义
         w
         r
         v
         twist
         offset
         qlim
         qdlim
         qddlim
         torlim
         theta
         serialtype  %机器人类型,如'RRRRRR'

     end
     
     methods
         function robot=SerialManu(varargin)
             %创建串联机械臂对象
             % r=SerialManu()用于创建一个空的对象
             %其中各项参数选取为默认值
             %
             %r=SerialManu(robot)用于复制一个机器人对象
             %会完全复制它的属性
             %
             %r=SerialManu([jl1 jl2 ... ]，option)利用参数创建串联机械臂
             %[jl1 jl2 ... ]表示关节连杆依次摆放
             %
             %options:
             %'name',name            机器人名称
             %'comment',comment      机器人的相关说明，例如机器人厂商，时间，特性等等
             %'base',T               设定机器人基座的变换矩阵（4X4double or SE3）
             %'tool',T               设定机器人的工具坐标系变换（4X4double or SE3）
             %'gravity',G            重力参数,必须为三维向量
             %plot3dopt,p               绘制三维模型的方式（n无法绘制，y可以绘制）
             
             %设置默认参数
             opt.jolinks=[];
             opt.n=0;
             opt.name = '';
             opt.comment ='';
             opt.base = eye(4);
             opt.tool = eye(4);
             opt.gravity = [0; 0; 9.81];
             opt.T0=[];
             opt.plot3dopt = {};
             opt.ikine = {};

             
             [opt,arg] = tb_optparse(opt, varargin);
                 
             if isempty(varargin)
                 %以默认参数创建默认对象;
                 robot.n=0;
                 robot.jolinks=[];
                 robot.name='';
                 robot.comment = '';
                 robot.T0=[];
                 robot.base = eye(4);
                 robot.tool=eye(4);
                 robot.gravity=[0; 0; 9.81];
                 robot.plot3dopt={};
             elseif nargin==1 && isa(varargin{1},'SerialManu')
                 %复制串联机器人对象
                 this = varargin{1};
                 robot.n=this.n;
                 robot.jolinks=this.jolinks;
                 robot.name=this.name;
                 robot.comment = this.comment;
                 robot.T0=this.T0;
                 robot.base = this.base;
                 robot.tool=this.tool;
                 robot.gravity=this.gravity;       
                 robot.plot3dopt=this.gravity;
             elseif nargin>=2 &&  isa(arg{1},'JoLink')
                 %根据参数创建串联机器人
                 Link=arg{1};
                 robot.jolinks=Link;
                 robot.n=length(robot.jolinks);
                 robot.name=opt.name;
                 robot.comment = opt.comment;
                 robot.T0=opt.T0;
                 robot.base = opt.base;
                 robot.tool=opt.tool;
                 robot.gravity=opt.gravity;
                 if isempty(robot.T0)
                     error('请输入机器人末端初始位置T0')
                 end
             else
                 error('输入格式有误');
             end
             
         end%构造函数
         
         
         function display(robot)
             %输出机器人参数
             if robot.n==0 || isempty(robot.n)
                 disp('    该机器人为空！！！')
             end
             disp([robot.name,' : ',num2str(robot.n),'axis , ',robot.serialtype]);
             disp(robot.comment)
             disp('+---+---------------------------+---------------------------+-----------+')
             disp('| j |             w             |              v            |   offset  |')
             disp('+---+---------------------------+---------------------------+-----------+')
             for i=1:robot.n
%                  
                 disp(['| ',num2str(i),' | ',sprintf('%.4f\t',robot.jolinks(i).w'),'| ',sprintf('%.4f\t',robot.jolinks(i).v'),'| ',...
                     sprintf('%.4f\t',robot.jolinks(i).offset),'|']);
             end
             disp('+---+---------------------------+---------------------------+-----------+')
             disp('T0:');
             disp(robot.T0);
             if ~isequal(robot.base,eye(4))
                 disp('base:');
                 disp(robot.base);
             end
             if ~isequal(robot.tool,eye(4))
                 disp('tool:');
                 disp(robot.tool);
             end
         end%输出
         
         %为从属属性定义set函数
         function k=get.w(robot)
             if robot.n==0
                 k=[];
             else
                 
                 k = [robot.jolinks.w]';
             end
         end %w属性
         
         function k=get.v(robot)
             if robot.n==0
                 k=[];
             else
                 k = [robot.jolinks.v]';
             end
         end %v属性
         
         function k=get.offset(robot)
             if robot.n==0
                 k=[];
             else
                 k = [robot.jolinks.offset];
             end
         end %offset属性
         
         function k=get.qlim(robot)
             if robot.n==0
                 k=[];
             else
                 for i=1:robot.n
                     k(i,:)=robot.jolinks(i).qlim;
                 end
             end
         end %qlim属性
         
         function k=get.qdlim(robot)
             if robot.n==0
                 k=[];
             else
                 k = [robot.jolinks.qdlim];
             end
         end %qdlim属性
         
         function k=get.qddlim(robot)
             if robot.n==0
                 k=[];
             else
                 k = [robot.jolinks.qddlim];
             end
         end %qddlim属性
         
         function k=get.torlim(robot)
             if robot.n==0
                 k=[];
             else
                 k = [robot.jolinks.torlim];
             end
         end %w属性
         
         function k=get.twist(robot)
             k=[robot.v robot.w];
         end %w属性
         
         function k=get.serialtype(robot)
             if robot.n==0
                 k=[];
             else
                 k = [robot.jolinks.jointtype];
             end
         end %w属性
         
         function base=get.base(robot)
             if isa(robot.base,'SE3')
                 base=double(robot.base);
             elseif isempty(robot.base)
                 base=eye(4);
             elseif size(robot.base)~=[4,4] | det(robot.base)~=1
                 error('base输入有误')
             else
                 base=robot.base;
             end
         end
         
         function tool=get.tool(robot)
             if isa(robot.tool,'SE3')
                 tool=double(robot.tool);
             elseif isempty(robot.tool)
                 tool=eye(4);
             elseif size(robot.tool)~=[4,4] | det(robot.tool)~=1
                 error('tool输入有误')
             else
                 tool=robot.tool;
             end
         end
         
         function T0=get.T0(robot)
             if isa(robot.T0,'SE3')
                 T0=double(robot.T0);
             elseif isempty(robot.T0)
                 T0=eye(4);
             elseif ~(size(robot.T0)==[4,4]) | det(robot.T0)~=1
                 error('T0输入有误')
             else
                 T0=robot.T0;
             end
         end
         
         function gravity=get.gravity(robot)
             if size(robot.gravity)==[1,3]
                 gravity=robot.gravity';
             elseif isempty(robot.gravity)
                 gravity=[0;0;9.81];
             elseif size(robot.gravity)~=[3,1]
                 error('gravity 输入有误')
             end
         end
     end%methods
end%类