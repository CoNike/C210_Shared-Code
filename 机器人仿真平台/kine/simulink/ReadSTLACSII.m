function [fout, vout, cout] = ReadSTLACSII(filename)


fid=fopen(filename, 'r');
if fid == -1
        error('文件打开错误！.')
end


%读取文件头，STL文件第一行是文件名
File_name = sscanf(fgetl(fid), '%*s %s'); %CAD object name, if needed.


%定义变量
vnum=0;  %Vertex number counter.
report_num=0; %Report the status as we go.
STLvcolor = 0;
STLxyz=0;


%开始读整个文件，直到结束符为止
while feof(fid) == 0 % test for end of file, if not then do stuff
        tline = fgetl(fid);  % reads a line of data from file.
        fword = sscanf(tline, '%s');  % make the line a character string


%检查颜色
if strncmpi(fword, 'facet normal',12) == 1 % Checking if a "C"olor line, as "C" is 1st char.
        STLvcolor = sscanf(tline, '%*s %f %f %f'); % & if a C, get the RGB color data of the face. ? % we "*s" skip the name "color" and get the data. ? ? ?
end % Keep this color, until the next color is used.


%检查坐标
if strncmpi(fword, 'vertex',6) == 1 % Checking if a "V"ertex line, as "V" is 1st char.
        STLxyz= sscanf(tline, '%*s %lf %lf %lf'); % & if a V, get the XYZ data of it.
        vnum = vnum + 1;
end


%检查坐标是否增加
if vnum~=report_num
        report_num=vnum;
         v(:,vnum)=STLxyz;
         c(:,vnum) = STLvcolor;% A color for each vertex, which will color the?
        if mod(report_num,249)==0;
        disp(sprintf('Reading vertix num: %d.',vnum));
        end
end

end


%关闭文件
fclose(fid);




% ? Build face list; The vertices are in order, so just number them.
fnum = vnum/3; %Number of faces, vnum is number of vertices. ?STL is triangles.
flist = 1:vnum; %Face list of vertices, all in order.
F = reshape(flist, 3,fnum); %Make a "3 by fnum" matrix of face list data.
%
% ? Return the faces and vertexs.
%
fout = F';%Orients the array for direct use in patch.
vout = v';% "
cout = c';




end