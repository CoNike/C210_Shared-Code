%此函数用于将机器人的XML文件
%在使用时请务必添加函数parseXML
%path为文件路径和文件名

%robot输出为机器人结构体（SerialManu）
%XML文件的编写模板参考puma560.xml

%1.0版本
%读取运动学关节参数，生成SerialManu
%2.0版本 
%添加几何文件读取功能

%Creator:Huang Zhouzhou  Time：2019/09/23

function robot = xml2robot3d(path)
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
    if length(link)==4
        [~,offset_link,r_link,w_link]=link.Value;
        offset_link=str2num(offset_link);
        r_link=str2num(r_link);
        w_link=str2num(w_link);
        jolink(i)=JoLink('w',w_link,'r',r_link,'offset',offset_link);
    elseif length(link)==3
        [~,offset_link,v_link]=link.Value;
        offset_link=str2num(offset_link);
        v_link=str2num(v_link);
        jolink(i)=JoLink('v',v_link,'offset',offset_link);
    else
        error('XML文件有误')
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
for i=1:k_geo
    attr_geo=child_geo(i).Attributes;
    [n_link,~,file]=attr_geo.Value;%获取几何文件值
    n_link=str2num(n_link);%获取其对应的连杆i
    [p_link,f_link]=stlRead(file); %获取STL的面和点
    %存储数据
    P{n_link+1}=[P{n_link+1};p_link];
    F{n_link+1}=[F{n_link+1};f_link];
end
robot = SerialManu(jolink,'base',base,'tool',tool,'T0',T0,'name',name_robot,'points',P,'faces',F);
end