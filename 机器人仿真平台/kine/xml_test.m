% xml的读取

%% 1.0版本
% D1=parseXML('input.xml');
% D2=xmlread('Epson_G10.xml','r');
% p560xml = parseXML('puma560.xml');
% % D2=parseXML('Epson_G10.xml');
% type=p560xml.Name;
% name=p560xml.Attributes;
% if strcmp(name.Name,'name')
%     temp.name=name.Value;
% end
% children=p560xml.Children;
% n=length(children);
% k=0;
% for i=1:n
%     if ~isempty(children(i).Attributes)
%         k=k+1;
%         child2(k)=children(i);
%     end
% end
% n_child=length(child2);
% n_robot=n_child-3;
% base=child2(1);
% tool=child2(n_child-1);
% T0=child2(n_child);


%% 2.0版本
% p560xml = parseXML('puma560.xml');
% xml_children=p560xml.Children;
% frame=xml_children(2).Children;
% geometry=xml_children(4).Children;
% k=0;
% n=length(geometry);
% for i=1:n
%     if ~isempty(geometry(i).Attributes)
%         k=k+1;
%         child_geo(k)=geometry(i);
%     end
% end
% 
% for i=1:k
%     
% end
% p560xml = xml2robot3d('puma560.xml')

%% 2.1版本测试
% robot_xml = parseXML('puma560.xml'); %读取文件，其中存在一定的冗余
% robot_children=robot_xml.Children;
% type_robot=robot_xml.Name;%暂时不会使用该数据
% if strcmp(robot_xml.Attributes.Name,'name')
%     name_robot=robot_xml.Attributes.Value;
% else
%     error('格式有误')
% end
% if ~strcmp(robot_children(2).Name,'Frame')
%     error('输入XML格式有误')
% end
% frame=robot_children(2).Children;
% n_frame=length(frame);
% k_joink=0;
% for i=1:n_frame
%     if ~isempty(frame(i).Attributes)
%         k_joink=k_joink+1;
%         child_frame(k_joink)=frame(i);
%     end
% end
% %获取base
% [base_rpy,base_xyz]=child_frame(1).Attributes.Value;
% base_rpy=str2num(base_rpy);
% base_xyz=str2num(base_xyz);
% base=transl(base_xyz)*rpy2tr(base_rpy);
% 
% %获取tool
% [tool_rpy,tool_xyz]=child_frame(k_joink-1).Attributes.Value;
% tool_rpy=str2num(tool_rpy);
% tool_xyz=str2num(tool_xyz);
% tool=transl(tool_xyz)*rpy2tr(tool_rpy);
% 
% %获取T0
% [T0_rpy,T0_xyz]=child_frame(k_joink).Attributes.Value;
% T0_rpy=str2num(T0_rpy);
% T0_xyz=str2num(T0_xyz);
% T0=transl(T0_xyz)*rpy2tr(T0_rpy);
% 
% %获取连杆
% for i=1:k_joink-3
%     link=child_frame(i+1).Attributes;
%     link_name=cell(0,0);
%     link_value=cell(0,0);
%     link_length=length(link);
%     for j=1:link_length
%         link_name=[link_name;link(j).Name];
%         link_value=[link_value;link(j).Value];
%     end
%     if strcmp(link_name(link_length),'w')
%         if link_length==3
%             joint_r=str2num(link_value{2});
%             joint_w=str2num(link_value{3});
%             joint_offset=0;
%             joint_qlim=[-180 180];
%             joint_qdlim=135;
%             joint_qddlim=120;
%         elseif link_length==4
%             if strcmp(link_name{2},'offset')
%                 joint_r=str2num(link_value{3});
%                 joint_w=str2num(link_value{4});
%                 joint_offset=str2num(link_value{2});
%                 joint_qlim=[-180 180];
%                 joint_qdlim=135;
%                 joint_qddlim=120;
%             elseif strcmp(link_name{2},'qlim')
%                 joint_r=str2num(link_value{3});
%                 joint_w=str2num(link_value{4});
%                 joint_qlim=str2num(link_value{2});
%                 joint_offset=0;
%                 joint_qdlim=135;
%                 joint_qddlim=120;
%             else
%                 error('输入属性名称有误');
%             end
%         elseif link_length==5
%             joint_r=str2num(link_value{4});
%             joint_w=str2num(link_value{5});
%             joint_qlim=str2num(link_value{3});
%             joint_offset=str2num(link_value{2});
%             joint_qdlim=135;
%             joint_qddlim=120;
%         elseif link_length==7
%             joint_r=str2num(link_value{6});
%             joint_w=str2num(link_value{7});
%             joint_qlim=str2num(link_value{5});
%             joint_offset=str2num(link_value{2});
%             joint_qdlim=str2num(link_value{4});
%             joint_qddlim=str2num(link_value{3});
%         else
%             error('输入属性数目不对')
%         end
%         jolink(i)=JoLink('w',joint_w,'r',joint_r,'offset',joint_offset,'qlim',...
%             joint_qlim,'qdlim',joint_qdlim,'qddlim',joint_qddlim);
%     elseif strcmp(link_name(link_length),'v')
%         if link_length==2
%             joint_v=str2num(link_value{2});
%             joint_offset=0;
%             joint_qlim=0.3;%单位为m
%             joint_qdlim=1;
%             joint_qddlim=1;
%         elseif link_length==3
%             if strcmp(link_name{2},'offset')
%                 joint_v=str2num(link_value{3});
%                 joint_offset=str2num(link_value{2});
%                 joint_qlim=0.3;%单位为m
%                 joint_qdlim=1;
%                 joint_qddlim=1;
%             elseif strcmp(link_name{2},'qlim')
%                 joint_v=str2num(link_value{3});
%                 joint_offset=0;
%                 joint_qlimstr2num(link_value{2});%单位为m
%                 joint_qdlim=1;
%                 joint_qddlim=1;
%             else
%                 error('输入属性有误')
%             end
%         elseif link_length==4
%             joint_v=str2num(link_value{4});
%             joint_qlim=str2num(link_value{3});
%             joint_offset=str2num(link_value{2});
%             joint_qdlim=1;
%             joint_qddlim=1;
%         elseif link_length==6
%             joint_v=str2num(link_value{6});
%             joint_qlim=str2num(link_value{5});
%             joint_offset=str2num(link_value{2});
%             joint_qdlim=str2num(link_value{4});
%             joint_qddlim=str2num(link_value{3});
%         else
%             error('输入属性数目不对')
%         end
%         jolink(i)=JoLink('v',joint_v,'offset',joint_offset,'qlim',joint_qlim,'qdlim',joint_qdlim,'qddlim',joint_qddlim);
%     end
% 
% end
% n=length(jolink); %获取机器人连杆数
% 
% p560xml = xml2robot3d_21('puma560.xml')
 ur5 = xml2robot3d_21('UR5.xml')
%  scara=xml2robot3d_21('epsonG651S.xml')