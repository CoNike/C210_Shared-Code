%�˺������ڽ������˵�XML�ļ�
%��ʹ��ʱ�������Ӻ���parseXML
%pathΪ�ļ�·�����ļ���
%robot���Ϊ�����˽ṹ�壨SerialManu��
%XML�ļ��ı�дģ��ο�puma560.xml

%1.0�汾
%��ȡ�˶�ѧ�ؽڲ���������SerialManu
%2.0�汾 
%��Ӽ����ļ���ȡ����
%2.1�汾
%�����˶�ȡ�ؽ����ƵĹ���
%�����˶�������ķ�ʽ

%Creator:Huang Zhouzhou  Time��2019/12/4

function robot = xml2robot3d_21(path)
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
    link_name=cell(0,0);
    link_value=cell(0,0);
    link_length=length(link);
    for j=1:link_length
        link_name=[link_name;link(j).Name];
        link_value=[link_value;link(j).Value];
    end
    if strcmp(link_name(link_length),'w')
        if link_length==3      %ֻ����������w��r
            joint_r=str2num(link_value{2});
            joint_w=str2num(link_value{3});
            joint_offset=0;
            joint_qlim=[-180 180];
            joint_qdlim=135;
            joint_qddlim=120;
        elseif link_length==4 
            if strcmp(link_name{2},'offset')%ֻ����������w��r��offset
                joint_r=str2num(link_value{3});
                joint_w=str2num(link_value{4});
                joint_offset=str2num(link_value{2});
                joint_qlim=[-180 180];
                joint_qdlim=135;
                joint_qddlim=120;
            elseif strcmp(link_name{2},'qlim')%ֻ����������w��r��qlim
                joint_r=str2num(link_value{3});
                joint_w=str2num(link_value{4});
                joint_qlim=str2num(link_value{2});
                joint_offset=0;
                joint_qdlim=135;
                joint_qddlim=120;
            else
                error('����������������');
            end
        elseif link_length==5 %ֻ����������w��r��offset,qlim
            joint_r=str2num(link_value{4});
            joint_w=str2num(link_value{5});
            joint_qlim=str2num(link_value{3});
            joint_offset=str2num(link_value{2});
            joint_qdlim=135;
            joint_qddlim=120;
        elseif link_length==7%ֻ����������w��r��offset,qlim,qdlim,qddlim
            joint_r=str2num(link_value{6});
            joint_w=str2num(link_value{7});
            joint_qlim=str2num(link_value{5});
            joint_offset=str2num(link_value{2});
            joint_qdlim=str2num(link_value{4});
            joint_qddlim=str2num(link_value{3});
        else
            error('����������Ŀ����')
        end
        jolink(i)=JoLink('w',joint_w,'r',joint_r,'offset',joint_offset,'qlim',...
            joint_qlim,'qdlim',joint_qdlim,'qddlim',joint_qddlim);
    elseif strcmp(link_name(link_length),'v') %�ƶ��ؽ�
        if link_length==2 %ֻ����v
            joint_v=str2num(link_value{2});
            joint_offset=0;
            joint_qlim=0.3;         %��λΪm
            joint_qdlim=1;
            joint_qddlim=1;
        elseif link_length==3       %����v��offset
            if strcmp(link_name{2},'offset')
                joint_v=str2num(link_value{3});
                joint_offset=str2num(link_value{2});
                joint_qlim=0.3;%��λΪm
                joint_qdlim=1;
                joint_qddlim=1;
            elseif strcmp(link_name{2},'qlim')      %����v��qlim
                joint_v=str2num(link_value{3});
                joint_offset=0;
                joint_qlimstr2num(link_value{2});%��λΪm
                joint_qdlim=1;
                joint_qddlim=1;
            else
                error('������������')
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
            error('����������Ŀ����')
        end
        jolink(i)=JoLink('v',joint_v,'offset',joint_offset,'qlim',joint_qlim,'qdlim',joint_qdlim,'qddlim',joint_qddlim);
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
n_temp=-1;%�����n_j��ͬȷ���ؽ�i��ͼ��
for i=1:k_geo
    attr_geo=child_geo(i).Attributes;
    [n_link,~,file]=attr_geo.Value;%��ȡ�����ļ�ֵ
    n_link=str2num(n_link);%��ȡ���Ӧ������i
    [p_link,f_link]=stlRead(file); %��ȡSTL����͵�
    %�洢����
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