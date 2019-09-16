%�˺������ڽ������˵�XML�ļ�
%��ʹ��ʱ�������Ӻ���parseXML
%pathΪ�ļ�·�����ļ���
%robot���Ϊ�����˽ṹ�壨SerialManu��
%XML�ļ��ı�дģ��ο�puma560.xml

function robot = xml2robot(path)
%% ��ȡ�ļ�
robot_xml = parseXML(path); %��ȡ�ļ������д���һ��������
%% ȥ��children�е�����
children=robot_xml.Children;
n=length(children);
k=0;
for i=1:n
    if ~isempty(children(i).Attributes)
        k=k+1;
        child(k)=children(i);
    end
end
%% ��ȡ��ز���
% n_chlid=length(children);%��ȡԪ������

%��ȡ���������ͺͻ���������
%��ʱ����������ֻ�д��������ˣ���Ϊû������
type_robot=robot_xml.Name;
if strcmp(robot_xml.Attributes.Name,'name')
    name_robot=robot_xml.Attributes.Value;
else
    error('��ʽ����')
end
%��ȡbase
[base_rpy,base_xyz]=child(1).Attributes.Value;
base_rpy=str2num(base_rpy);
base_xyz=str2num(base_xyz);
base=transl(base_xyz)*rpy2tr(base_rpy);

%��ȡtool
[tool_rpy,tool_xyz]=child(k-1).Attributes.Value;
tool_rpy=str2num(tool_rpy);
tool_xyz=str2num(tool_xyz);
tool=transl(tool_xyz)*rpy2tr(tool_rpy);

%��ȡT0
[T0_rpy,T0_xyz]=child(k).Attributes.Value;
T0_rpy=str2num(T0_rpy);
T0_xyz=str2num(T0_xyz);
T0=transl(T0_xyz)*rpy2tr(T0_rpy);

%��ȡ����
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
        error('XML�ļ�����')
    end
end

%% ���ɻ�����
robot = SerialManu(jolink,'base',base,'tool',tool,'T0',T0,'name',name_robot);
end