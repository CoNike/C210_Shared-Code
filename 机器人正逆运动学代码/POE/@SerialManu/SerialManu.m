%��������ָ������ʽ�Ĵ���������ģ��
%��������Ӧ�ò��������˺����λ����˵Ĵ���
%robot=SerialManu();����һ���յĻ�����
%robot=SerialManu(robot);��������һ�������ˣ��Ḵ����ͬ�Ĳ���
%robot=SerialManu([jolink1 jolink2],'T0',4X4double,...(option))
%���ò�������������ģ��
%����[jolink1 jolink2]ΪJoLink�����������嶨����Բο�JoLink��
%option:
%'T0',4X4double��SE3��������ĩ��ִ�����ĳ�ʼλ��,���븳ֵ
%'name',char,���������ƣ�default,''��
%'comment',char,���������˵����default,''��
%'gravity',3X1double,�������������ٶȷ���default,[0 0 9.81]��
%'plot3dopt',char,������3D���Ƶķ�ʽ��default,{}����ʱ��δ����
%ikineType ��char,���������˶�ѧ���ͣ�default,{}����ʱ��δ����

%����ָ���������ݿ��Բο�һ�����ף�
%�����׵ȡ�������ѧ����ģ�������Ӿ���
%������ȡ�������ѧ�ļ��λ�����
%������Բο�Peter Corke��RTB������

%creator: Huang Zhouzhou  Time:2019/9/6
%Huazhong University of Science and Technology
classdef SerialManu < handle 
     properties
         name       %����������
         comment    %���������˵��
         T0         %�趨������ĩ������ڻ�����ϵ�ĳ�ʼλ��
         gravity    %�������ٶ�ʸ��
         base       %�����˻�������ϵ�任
         tool       %�����˹�������ϵ�任
         plot3dopt     %3d����
         ikineType  %�������
         faces      %��ȡ�����ļ�֮�����͵�
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
         %�˴�Ϊ�������ԣ��������Բ���ͨ����ֵ����
         %ֻ����get��������
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
         serialtype  %����������,��'RRRRRR'

     end
     
     methods
         function robot=SerialManu(varargin)
             %����������е�۶���
             % r=SerialManu()���ڴ���һ���յĶ���
             %���и������ѡȡΪĬ��ֵ
             %
             %r=SerialManu(robot)���ڸ���һ�������˶���
             %����ȫ������������
             %
             %r=SerialManu([jl1 jl2 ... ]��option)���ò�������������е��
             %[jl1 jl2 ... ]��ʾ�ؽ��������ΰڷ�
             %
             %options:
             %'name',name            ����������
             %'comment',comment      �����˵����˵������������˳��̣�ʱ�䣬���Եȵ�
             %'base',T               �趨�����˻����ı任����4X4double or SE3��
             %'tool',T               �趨�����˵Ĺ�������ϵ�任��4X4double or SE3��
             %'gravity',G            ��������,����Ϊ��ά����
             %plot3dopt,p               ������άģ�͵ķ�ʽ��n�޷����ƣ�y���Ի��ƣ�
             
             %����Ĭ�ϲ���
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
                 %��Ĭ�ϲ�������Ĭ�϶���;
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
                 %���ƴ��������˶���
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
                 %���ݲ�����������������
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
                     error('�����������ĩ�˳�ʼλ��T0')
                 end
             else
                 error('�����ʽ����');
             end
             
         end%���캯��
         
         
         function display(robot)
             %��������˲���
             if robot.n==0 || isempty(robot.n)
                 disp('    �û�����Ϊ�գ�����')
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
         end%���
         
         %Ϊ�������Զ���set����
         function k=get.w(robot)
             if robot.n==0
                 k=[];
             else
                 
                 k = [robot.jolinks.w]';
             end
         end %w����
         
         function k=get.v(robot)
             if robot.n==0
                 k=[];
             else
                 k = [robot.jolinks.v]';
             end
         end %v����
         
         function k=get.offset(robot)
             if robot.n==0
                 k=[];
             else
                 k = [robot.jolinks.offset];
             end
         end %offset����
         
         function k=get.qlim(robot)
             if robot.n==0
                 k=[];
             else
                 for i=1:robot.n
                     k(i,:)=robot.jolinks(i).qlim;
                 end
             end
         end %qlim����
         
         function k=get.qdlim(robot)
             if robot.n==0
                 k=[];
             else
                 k = [robot.jolinks.qdlim];
             end
         end %qdlim����
         
         function k=get.qddlim(robot)
             if robot.n==0
                 k=[];
             else
                 k = [robot.jolinks.qddlim];
             end
         end %qddlim����
         
         function k=get.torlim(robot)
             if robot.n==0
                 k=[];
             else
                 k = [robot.jolinks.torlim];
             end
         end %w����
         
         function k=get.twist(robot)
             k=[robot.v robot.w];
         end %w����
         
         function k=get.serialtype(robot)
             if robot.n==0
                 k=[];
             else
                 k = [robot.jolinks.jointtype];
             end
         end %w����
         
         function base=get.base(robot)
             if isa(robot.base,'SE3')
                 base=double(robot.base);
             elseif isempty(robot.base)
                 base=eye(4);
             elseif size(robot.base)~=[4,4] | det(robot.base)~=1
                 error('base��������')
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
                 error('tool��������')
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
                 error('T0��������')
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
                 error('gravity ��������')
             end
         end
     end%methods
end%��