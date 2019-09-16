%此函数用于将机器人的XML文件
%在使用时请务必添加函数parseXML
%path为文件路径和文件名
%robot输出为机器人结构体（SerialManu）
%XML文件的编写模板参考puma560.xml

function robot = xml2robot(path)
%% 读取文件
robot_xml = parseXML(path); %读取文件，其中存在一定的冗余
%% 去除children中的冗余
children=robot_xml.Children;
n=length(children);
k=0;
for i=1:n
    if ~isempty(children(i).Attributes)
        k=k+1;
        child(k)=children(i);
    end
end
%% 获取相关参数
% n_chlid=length(children);%获取元素总数

%获取机器人类型和机器人名称
%暂时机器人类型只有串联机器人，因为没有设置
type_robot=robot_xml.Name;
if strcmp(robot_xml.Attributes.Name,'name')
    name_robot=robot_xml.Attributes.Value;
else
    error('格式有误')
end
%获取base
[base_rpy,base_xyz]=child(1).Attributes.Value;
base_rpy=str2num(base_rpy);
base_xyz=str2num(base_xyz);
base=transl(base_xyz)*rpy2tr(base_rpy);

%获取tool
[tool_rpy,tool_xyz]=child(k-1).Attributes.Value;
tool_rpy=str2num(tool_rpy);
tool_xyz=str2num(tool_xyz);
tool=transl(tool_xyz)*rpy2tr(tool_rpy);

%获取T0
[T0_rpy,T0_xyz]=child(k).Attributes.Value;
T0_rpy=str2num(T0_rpy);
T0_xyz=str2num(T0_xyz);
T0=transl(T0_xyz)*rpy2tr(T0_rpy);

%获取连杆
for i=1:k-3
    link=child(i+1).Attributes;
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

%% 生成机器人
robot = SerialManu(jolink,'base',base,'tool',tool,'T0',T0,'name',name_robot);
end