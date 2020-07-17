%�˺������ڽ������˵�XML�ļ�
%��ʹ��ʱ�������Ӻ���parseXML
%pathΪ�ļ�·�����ļ���

%robot���Ϊ�����˽ṹ�壨SerialManu��
%XML�ļ��ı�дģ��ο�puma560.xml

%1.0�汾
%��ȡ�˶�ѧ�ؽڲ���������SerialManu
%2.0�汾 
%��Ӽ����ļ���ȡ����

%Creator:Huang Zhouzhou  Time��2019/09/23

function robot = xml2robot3d(path)
%% ��ȡ�ļ�
robot_xml = parseXML(path); %��ȡ�ļ������д���һ��������
robot_children=robot_xml.Children;
type_robot=robot_xml.Name;%��ʱ����ʹ�ø�����
if strcmp(robot_xml.Attributes.Name,'name')
    name_robot=robot_xml.Attributes.Value;
else
    error('��ʽ����')
end

%% ��������
if ~strcmp(robot_children(2).Name,'Frame')
    error('����XML��ʽ����')
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
%��ȡbase
[base_rpy,base_xyz]=child_frame(1).Attributes.Value;
base_rpy=str2num(base_rpy);
base_xyz=str2num(base_xyz);
base=transl(base_xyz)*rpy2tr(base_rpy);

%��ȡtool
[tool_rpy,tool_xyz]=child_frame(k_joink-1).Attributes.Value;
tool_rpy=str2num(tool_rpy);
tool_xyz=str2num(tool_xyz);
tool=transl(tool_xyz)*rpy2tr(tool_rpy);

%��ȡT0
[T0_rpy,T0_xyz]=child_frame(k_joink).Attributes.Value;
T0_rpy=str2num(T0_rpy);
T0_xyz=str2num(T0_xyz);
T0=transl(T0_xyz)*rpy2tr(T0_rpy);

%��ȡ����
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
        error('XML�ļ�����')
    end
end
n=length(jolink); %��ȡ������������
%% ��ȡ����ͼ���ļ���
if ~strcmp(robot_children(4).Name,'geometry')
    error('����XML��ʽ����')
end
geometry=robot_children(4).Children; %��ȡ����ͼ�ε��ӽṹ��
k_geo=0;
n_geo=length(geometry); %��ȡ�ṹ��ʵ�ʳ���
%��⼸��ͼ�νṹ���зǿ��ӽṹ������������
for i=1:n_geo
    if ~isempty(geometry(i).Attributes)
        k_geo=k_geo+1;  
        child_geo(k_geo)=geometry(i);
    end
end
%ȷ�������˻�ͼʱ�����͵�
P=cell(1,n+1);
F=cell(1,n+1);
for i=1:k_geo
    attr_geo=child_geo(i).Attributes;
    [n_link,~,file]=attr_geo.Value;%��ȡ�����ļ�ֵ
    n_link=str2num(n_link);%��ȡ���Ӧ������i
    [p_link,f_link]=stlRead(file); %��ȡSTL����͵�
    %�洢����
    P{n_link+1}=[P{n_link+1};p_link];
    F{n_link+1}=[F{n_link+1};f_link];
end
robot = SerialManu(jolink,'base',base,'tool',tool,'T0',T0,'name',name_robot,'points',P,'faces',F);
end