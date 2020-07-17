%此函数用于将机器人的XML文件
%在使用时请务必添加函数parseXML
%path为文件路径和文件名
%robot输出为机器人结构体（SerialManu）
%XML文件的编写模板参考puma560.xml

%1.0版本
%读取运动学关节参数，生成SerialManu
%2.0版本 
%添加几何文件读取功能
%2.1版本
%增加了读取关节限制的功能
%增加了多种输入的方式

%Creator:Huang Zhouzhou  Time：2019/12/4

function robot = xml2robot3d_21(path)
%% 读取文件
robot_xml = parseXML(path); %读取文件，其中存在一定的冗余
robot_children=robot_xml.Children;
type_robot=robot_xml.Name;%暂时不会使用该数据
if strcmp(robot_xml.Attributes.Name,'name')
    name_robot=robot_xml.Attributes.Value;
else
    error('格式有误')
end

%% 创建连杆
if ~strcmp(robot_children(2).Name,'Frame')
    error('输入XML格式有误')
end
frame=robot_children(2).Children;
n_frame=length(frame);
k_joink=0;
for i=1:n_frame
    if ~isempty(frame(i).Attributes)
        k_joink=k_joink+1;
        child_frame(k_joink)=frame(i);
    end
end
%获取base
[base_rpy,base_xyz]=child_frame(1).Attributes.Value;
base_rpy=str2num(base_rpy);
base_xyz=str2num(base_xyz);
base=transl(base_xyz)*rpy2tr(base_rpy);

%获取tool
[tool_rpy,tool_xyz]=child_frame(k_joink-1).Attributes.Value;
tool_rpy=str2num(tool_rpy);
tool_xyz=str2num(tool_xyz);
tool=transl(tool_xyz)*rpy2tr(tool_rpy);

%获取T0
[T0_rpy,T0_xyz]=child_frame(k_joink).Attributes.Value;
T0_rpy=str2num(T0_rpy);
T0_xyz=str2num(T0_xyz);
T0=transl(T0_xyz)*rpy2tr(T0_rpy);

%获取连杆
for i=1:k_joink-3
    link=child_frame(i+1).Attributes;
    link_name=cell(0,0);
    link_value=cell(0,0);
    link_length=length(link);
    for j=1:link_length
        link_name=[link_name;link(j).Name];
        link_value=[link_value;link(j).Value];
    end
    if strcmp(link_name(link_length),'w')
        if link_length==3      %只输入旋量的w和r
            joint_r=str2num(link_value{2});
            joint_w=str2num(link_value{3});
            joint_offset=0;
            joint_qlim=[-180 180];
            joint_qdlim=135;
            joint_qddlim=120;
        elseif link_length==4 
            if strcmp(link_name{2},'offset')%只输入旋量的w和r和offset
                joint_r=str2num(link_value{3});
                joint_w=str2num(link_value{4});
                joint_offset=str2num(link_value{2});
                joint_qlim=[-180 180];
                joint_qdlim=135;
                joint_qddlim=120;
            elseif strcmp(link_name{2},'qlim')%只输入旋量的w和r和qlim
                joint_r=str2num(link_value{3});
                joint_w=str2num(link_value{4});
                joint_qlim=str2num(link_value{2});
                joint_offset=0;
                joint_qdlim=135;
                joint_qddlim=120;
            else
                error('输入属性名称有误');
            end
        elseif link_length==5 %只输入旋量的w和r和offset,qlim
            joint_r=str2num(link_value{4});
            joint_w=str2num(link_value{5});
            joint_qlim=str2num(link_value{3});
            joint_offset=str2num(link_value{2});
            joint_qdlim=135;
            joint_qddlim=120;
        elseif link_length==7%只输入旋量的w和r和offset,qlim,qdlim,qddlim
            joint_r=str2num(link_value{6});
            joint_w=str2num(link_value{7});
            joint_qlim=str2num(link_value{5});
            joint_offset=str2num(link_value{2});
            joint_qdlim=str2num(link_value{4});
            joint_qddlim=str2num(link_value{3});
        else
            error('输入属性数目不对')
        end
        jolink(i)=JoLink('w',joint_w,'r',joint_r,'offset',joint_offset,'qlim',...
            joint_qlim,'qdlim',joint_qdlim,'qddlim',joint_qddlim);
    elseif strcmp(link_name(link_length),'v') %移动关节
        if link_length==2 %只输入v
            joint_v=str2num(link_value{2});
            joint_offset=0;
            joint_qlim=0.3;         %单位为m
            joint_qdlim=1;
            joint_qddlim=1;
        elseif link_length==3       %输入v和offset
            if strcmp(link_name{2},'offset')
                joint_v=str2num(link_value{3});
                joint_offset=str2num(link_value{2});
                joint_qlim=0.3;%单位为m
                joint_qdlim=1;
                joint_qddlim=1;
            elseif strcmp(link_name{2},'qlim')      %输入v和qlim
                joint_v=str2num(link_value{3});
                joint_offset=0;
                joint_qlimstr2num(link_value{2});%单位为m
                joint_qdlim=1;
                joint_qddlim=1;
            else
                error('输入属性有误')
            end
        elseif link_length==4
            joint_v=str2num(link_value{4});
            joint_qlim=str2num(link_value{3});
            joint_offset=str2num(link_value{2});
            joint_qdlim=1;
            joint_qddlim=1;
        elseif link_length==6
            joint_v=str2num(link_value{6});
            joint_qlim=str2num(link_value{5});
            joint_offset=str2num(link_value{2});
            joint_qdlim=str2num(link_value{4});
            joint_qddlim=str2num(link_value{3});
        else
            error('输入属性数目不对')
        end
        jolink(i)=JoLink('v',joint_v,'offset',joint_offset,'qlim',joint_qlim,'qdlim',joint_qdlim,'qddlim',joint_qddlim);
    end
end
n=length(jolink); %获取机器人连杆数
%% 读取几何图形文件名
if ~strcmp(robot_children(4).Name,'geometry')
    error('输入XML格式有误')
end
geometry=robot_children(4).Children; %获取几何图形的子结构体
k_geo=0;
n_geo=length(geometry); %获取结构体实际长度
%求解几何图形结构体中非空子结构的数量和数据
for i=1:n_geo
    if ~isempty(geometry(i).Attributes)
        k_geo=k_geo+1;  
        child_geo(k_geo)=geometry(i);
    end
end
%确定机器人绘图时候的面和点
P=cell(1,n+1);
F=cell(1,n+1);
n_temp=-1;%与参数n_j共同确定关节i的图形
for i=1:k_geo
    attr_geo=child_geo(i).Attributes;
    [n_link,~,file]=attr_geo.Value;%获取几何文件值
    n_link=str2num(n_link);%获取其对应的连杆i
    [p_link,f_link]=stlRead(file); %获取STL的面和点
    %存储数据
    if n_link~=n_temp
        n_j=1;
        P{n_link+1}=p_link;
        F{n_link+1}=f_link;
    else
        n_j=n_j+1;
        if n_j==2
            P{n_link+1}={P{n_link+1},p_link};
            F{n_link+1}={F{n_link+1},f_link};
        else
            P{n_link+1}={P{n_link+1}{:},p_link};
            F{n_link+1}={F{n_link+1}{:},f_link};
        end
    end
    n_temp=n_link;
end
robot = SerialManu(jolink,'base',base,'tool',tool,'T0',T0,'name',name_robot,'points',P,'faces',F);
end