%% poefig相关函数记录与说明
%condition_number_button_press 条件数显示按钮  用以在图案中显示某条件轨迹的条件数
%condition_number_edit_button_press 条件数编辑控件
%manipulability_button_press 可操作度显示按钮 用以在图案中显示某条件轨迹的条件数
%manipulability_edit_button_press 可操作度显示按钮
%screwi_edit_button_press 第i个旋量的显示按钮
%inputjoint_button_press 输入角度轨迹函数按钮
%inputdescar_button_press 输入末端执行器轨迹函数按钮
%tci_edit_button_press  参考角度编辑控件
%tdx_edit_button_press,tdy_edit_button_press,tdz_edit_button_press末端执行器位置（x,y,z）编辑控件
%tdr_edit_button_press，tdp_edit_button_press，tdyo_edit_button_press末端执行器角度（rpy）编辑控件
% ikine_button_press逆运动学求解函数
% tci_edit_button_press'%参考角度函数
%ti_slider_button_press，ti_edit_button_press 关节i角度滑条控件和编辑控件

%% 数据存储说明
% 存储轨迹位置
% setappdata(0,'xtrail',0);
% setappdata(0,'ytrail',0);
% setappdata(0,'ztrail',0)

% setappdata(0,'Tr',Tr); %末端位置绘图

% setappdata(0,'fig0',fig); %绘图指定坐标轴

% setappdata(0,'ThetaOld',theta); %上一个状态的关节角度

%末端执行器位置和姿态
% setappdata(0,'descar_x',x_value);
% setappdata(0,'descar_y',y_value);
% setappdata(0,'descar_z',z_value);
% setappdata(0,'descar_r',r_value);
% setappdata(0,'descar_p',p_value);
% setappdata(0,'descar_yo',yo_value);
%参考角度
% setappdata(0,'tc1',tc1_value);
% setappdata(0,'tc2',tc2_value);
% setappdata(0,'tc3',tc3_value);
% setappdata(0,'tc4',tc4_value
% setappdata(0,'tc5',tc5_value);
% setappdata(0,'tc6',tc6_value);
%% 需要加入的功能
%1.根据机器人的大小绘图做出调整

%2.旋量显示，运动学性能未显示

%3.状态空间导入机器人模型

%4.逆运动学性能需要完善

%% 辅助说明
     %图形绘制的另一种方式
%     F00=getappdata(0,'F0');
%     P00=getappdata(0,'P0');
%     L0=patch('Faces',F00,'Vertices',P00);
%     set(L0,'FaceColor',[1 0 0],'EdgeColor','none');
%     H(1)=L0;
%     for i=1:n
%         temp1=getappdata(0,char(['F',num2str(i)]));
%         eval(['F',num2str(i),'0','=','temp1',';']);
%         temp2=getappdata(0,char(['P',num2str(i)]));
%         eval(['P',num2str(i),'0','=','temp2',';']);
%         temp3=patch('Faces',eval(['F',num2str(i),'0']),'Vertices',eval(['P',num2str(i),'0']));
%         eval(['L',num2str(i),'0=','temp3;']);
%         H(i+1)=eval(['L',num2str(i),'0']);
%     end
%     Tr = plot3(0,0,0,'b.');
%     H=[H,Tr];
%     setappdata(0,'patch_h',H);
%     setappdata(0,'ThetaOld',zeros(1,n));

%目前关节角度空间设置为8，感觉