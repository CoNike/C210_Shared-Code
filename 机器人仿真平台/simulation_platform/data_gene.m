%数据生成函数

%读取文件
[filename, pathname] = uigetfile({'*.mat;*.txt'},'File Selector');
% 获取文件后缀 
[~,~,c]=fileparts(filename);

if strcmp(c,'.txt')
    data=load([pathname filename]);
elseif strcmp(c,'.mat')
    data_temp=load([pathname filename]);
    data=data_temp.data;
else
    error('读入文件有误')
end

judge1 = input('是否为关节空间轨迹？（y/n）','s');
if strcmp(judge1,'y')
    [row,col]=size(data);
    if col~=6
        error('输入角度文件维数不对')
    end
    judge2 = input('角度是否为弧度制（y/n）','s');
    if strcmp(judge2,'n')
        data=data*pi/180;
    end
    trail = input('是否显示末端轨迹（y/n）','s');
else
    [row,col]=size(data);
    if col~=6
        error('输入末端位置文件维数不对')
    end
    judge2 = input('角度是否为弧度制（y/n）','s');
    if strcmp(judge2,'n')
        data=data*pi/180;
    end
    trail = input('是否显示末端轨迹（y/n）','s');
end

