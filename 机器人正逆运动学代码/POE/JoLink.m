%指数积模型中连杆的建立
%关于指数积公式的相关内容参考文献如下：
%熊有伦等《机器人学：建模控制与视觉》
%李泽湘等《机器人学的几何基础》
%程序参考了Peter Corke的RTB工具箱
%
%黄洲洲，2019.9.5

classdef JoLink < matlab.mixin.Copyable
    properties
        
        %运动学参数
        w   %关节轴线位置
        r   %关节轴线上一点
        v   %关节轴线矩
        theta   %关节变量
        jointtype  %关节类型，revolute='R', prismatic='P' -- should be an enum
        offset%关节默认偏角
        name  %连杆名字
        flip %关节反向移动
        qlim %关节角度限制(2x1)
        qdlim %关节角速度限制
        qddlim %关节角加速度限制
        % 动力学参数
        m  % l连杆质量
        rc % 连杆质心相对于基坐标系的位置
        I  % 连杆相对于自身质心的惯性矩阵
        %电机参数
        Jm % 电机等效转动惯量
        B  % 电机粘性摩擦系数 (1x1 or 2x1)
        torlim %关节力矩限制
        Tc % 电机库伦摩擦系数 (1x2 or 2x1)
        G  % 电机减速比  
        %模型参数
        stl
    end %类的性质

    methods
        function jl=JoLink(varargin)
            %JoLink的构造函数，用于创建连杆对象
            %
            %创建方式如下：
            %jl=JoLink();按照默认参数创建连杆
            %jl=JoLink(lolink);复制其他连杆对象
            %jl=JoLink(options);根据运动学和动力学参数创建连杆
            %options::
            %'w',w;关节轴线在基坐标系下的方向向量
            %'r',r;关节轴线上一点在基坐标系中的位置
            %'offset',o;默认起始关节角，用于调整使得模型与实物角度一致(defult 0)
            %'flip',logic;为true时关节旋向相反，false一致(default false)
            %'qlim',[a,b];关节角度限制（1X2,default[-pi,pi]）
            %'qdlim',qd;关节角速度限制（default[]）
            %'qddlim',qd;关节角加速度限制（default[]）
            %'m'，m;连杆质量（default 0）
            %'rc',rc;连杆质心相对于基坐标系的位置向量（default []）
            %'I',i;连杆相对于其质心的惯性矩阵(default [])
            % 'Jm',J  ;电机转动惯量(default 0)
            % 'B',B  ;关节粘滞摩擦系数（default 0）
            %'torlim',tor %关节力矩限制
            % 'Tc',T  ;电机库仑摩擦系数 (1x1 or 2x1), (default [0 0])
            % 'G',G ；电机减速比(default 101)

            if nargin==0
                %创建默认连杆

                %运动学参数
                jl.w=[0 0 1]';
                jl.r=[0 0 0]';
                jl.v=[0 0 0]';
                jl.theta=0;
                jl.offset=0;
                jl.flip=false;
                jl.name='';
                jl.qlim=[-pi pi];
                jl.qdlim=pi/2;%rad/s
                jl.qddlim=0.5;%rad/s^2
                jl.jointtype='R';
                
                %动力学参数
                jl.m=0;
                jl.rc=[0 0 0]';
                jl.I=zeros(3);
                
                %电机参数
                jl.Jm=0;
                jl.B=0;
                jl.torlim=0;
                jl.Tc=0;
                jl.G=101;
                %模型参数
                jl.stl={};

            elseif nargin==1 && isa(varargin{1},'JoLink')  %需要提取出元胞数组中的连杆
                %复制对象
                this = varargin{1};
                for j=1:length(this)
                    jl(j) = JoLink();
                    %复制属性
                    p = properties(this(j));
                    for i = 1:length(p)
                        jl(j).(p{i}) = this(j).(p{i});
                    end
                end
            else
                %根据给定参数创建连杆
                %设定相关参数
                opt.w= [];
                opt.r = [];
                opt.theta=[];
                opt.v=[];                
                opt.G = 0;                
                opt.B = 0;                
                opt.Tc = [0 0];                
                opt.Jm = 0;                
                opt.I = zeros(3,3);                
                opt.m = 0;                
                opt.rc = [0 0 0];
                opt.offset = 0;
                opt.qlim = [-pi pi];
                opt.qdlim =-pi/2;
                opt.qddlim =0.5;
                opt.type = {'revolute', 'prismatic', 'fixed'};
                opt.flip = [];
                opt.torlim=0;
                opt.stl={};
                [opt,~] = tb_optparse(opt, varargin);
                
                if length(varargin)>=1
                    %判断连杆类型
                    if isempty(opt.w)
                        if isempty(opt.v)
                            error('连杆参数有误，创建失败')
                        end
                        jl.jointtype='P';
                        jl.w=[0 0 0]';
                        jl.r=[0 0 0]';
                        jl.v=opt.v(:);
                        jl.v=jl.v/norm(jl.v);
                    else
                        if isempty(opt.w) || isempty(opt.r)
                            error('连杆参数有误，创建失败')
                        end
                        jl.jointtype='R';
                        jl.w= opt.w(:);
                        jl.w=jl.w/norm(jl.w);
                        jl.r= opt.r(:);
                        jl.v=cross(jl.r,jl.w);
                    end

                    jl.offset =opt.offset;
                    jl.flip =opt.flip;
                    
                    jl.qlim =  opt.qlim;
                    jl.qdlim =  opt.qdlim;
                    jl.qddlim =   opt.qddlim;
                    
                    jl.m =opt.m;
                    jl.rc = opt.rc;
                    jl.I = opt.I;
                    jl.Jm =opt.Jm;
                    jl.G =opt.G;
                    jl.B =opt.B;
                    jl.Tc = opt.Tc;
                    jl.torlim=opt.torlim;
                    jl.stl=opt.stl;
                else
                    error('输入为空')
                end
            end
        end%构造函数
        
        function tor=friction(jl,qd)
            %关节摩擦力求解
            %参考Link中同名函数编写
            tor = jl.B * abs(jl.G) * qd;
            % Coulomb friction
            if ~isa(qd, 'sym')
                if qd > 0
                    tor = tor + jl.Tc(1);
                elseif qd < 0
                    tor = tor + jl.Tc(2);
                end
            end
            % scale up by gear ratio
            tor = -abs(jl.G) * tor;     % friction opposes motion
        end % 摩擦力
        
        function logic=islimt(jl,q,qd,qdd,tor)
            %用于判断关节角度，关节角速度，角加速度和力是否超出限制
            logic1=0;
            if q>jl.qlim(2) || q<jl.qlim(1)
                logic1=logic1+1;
                fprintf('关节角度超出限制')
            end
            if abs(qd)>jl.qdlim 
                logic1=logic1+1;
                fprintf('关节角速度超出限制')
            end
            if abs(qdd)>jl.qddlim
                logic1=logic1+1;
                fprintf('关节角加速度超出限制')
            end
            if abs(tor)>jl.torlim
                logic1=logic1+1;
                fprintf('关节力矩超出限制')
            end
            if logic1>1
                logic=false;
            else
                logic=true;
            end
        end%判断是否超出限制
        
        function set.rc(jl,input)
            %设置连杆质心
            if isempty(input)
                jl.rc=[];
            elseif length(input)~=3
                error('输入参数有误')
            end
            jl.rc=input(:);
        end%设置rc
        
        function set.I(jl,input)
            %设置连杆转动惯量
            %input可以为3X3矩阵，3为数组（Ixx,Iyy,Izz）,6维向量
            %（Ixx,Iyy,Izz,Ixy,Ixz,Iyz）
            if all(size(input)==[3,3])
                jl.I=input;
            elseif length(input)==3
                jl.I=diag(input);
            elseif length(input)==6
                jl.I=[input(1) input(4) input(5);
                    input(4) input(2) input(6);
                    input(5) input(6) input(3)];
            elseif isempty(input)
                jl.I=[];
            else
                error('输入数据有误')
            end
        end%设置连杆转动惯量
        
        function set.Tc(jl,input)
            %设置电机摩擦系数，输入为1X2或者1X1
            if length(input)==1
                jl.Tc=[-abs(input) abs(input)];
            elseif length(input)==2
                jl.Tc=input;
            else
                error('输入数据有误')
            end
                
        end%设置关节摩擦系数
        
        function T=isom(jl,q)
            %求解刚体的变换矩阵
            %基于罗德里格斯公式
            %获取相关求解的变量
            w1=jl.w;
            v1=jl.v;
            if jl.flip==true
                q=-q+jl.offset;
            elseif jl.flip==false
                q=q+jl.offset;
            end
            if norm(w1)==0
                R=eye(3);
                t=v1*q;
            else
                R=eye(3)+skew0(w1)*sin(q)+skew0(w1)*skew0(w1)*(1-cos(q));
                t=(eye(3)-R)*skew(w1)*v1+w1*w1'*v1*q;
            end
            T=[R t;
                0 0 0 1];
            function w_skew=skew0(w)
                %自编的反对称矩阵，为区别与skew命名为skew0
                if length(w)~=3
                    error('输入数据有误')
                end
                w_skew=[0   -w(3)   w(2);
                        w(3) 0      -w(1);
                        -w(2)   w(1)    0];
            end%反对称矩阵
            
        end%关节刚体变换
        
        function display(jl)
            %输出关节连杆相关参数
            %
            disp('关节连杆是旋量方法建立');
            fprintf('%s : ',inputname(1));
            disp(['w =[',num2str(jl.w'),'] ' ,' v = [',num2str(jl.v'),'];']);
        end%display
        
    end %method
end %类的结束