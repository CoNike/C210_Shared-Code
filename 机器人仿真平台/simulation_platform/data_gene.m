%�������ɺ���

%��ȡ�ļ�
[filename, pathname] = uigetfile({'*.mat;*.txt'},'File Selector');
% ��ȡ�ļ���׺ 
[~,~,c]=fileparts(filename);

if strcmp(c,'.txt')
    data=load([pathname filename]);
elseif strcmp(c,'.mat')
    data_temp=load([pathname filename]);
    data=data_temp.data;
else
    error('�����ļ�����')
end

judge1 = input('�Ƿ�Ϊ�ؽڿռ�켣����y/n��','s');
if strcmp(judge1,'y')
    [row,col]=size(data);
    if col~=6
        error('����Ƕ��ļ�ά������')
    end
    judge2 = input('�Ƕ��Ƿ�Ϊ�����ƣ�y/n��','s');
    if strcmp(judge2,'n')
        data=data*pi/180;
    end
    trail = input('�Ƿ���ʾĩ�˹켣��y/n��','s');
else
    [row,col]=size(data);
    if col~=6
        error('����ĩ��λ���ļ�ά������')
    end
    judge2 = input('�Ƕ��Ƿ�Ϊ�����ƣ�y/n��','s');
    if strcmp(judge2,'n')
        data=data*pi/180;
    end
    trail = input('�Ƿ���ʾĩ�˹켣��y/n��','s');
end

