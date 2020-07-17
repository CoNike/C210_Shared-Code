%数据验证
% q0=[0 0 0 0 0 0];
% q1=[1 1 1 1 1 1 ];
% q=jtraj(q0,q1,50);
% q=roundn(q,-4);
% save D:\matlab\robotics-kine-dyn-controll-vision\simulation_platform\data\q.txt -ascii q
% 
%  fid = fopen('q1.txt','w');
%  data=q;
%  [m,n]=size(data);
%  for i=1:1:m
%      for j=1:1:n
%          if j==n
%              fprintf(fid,'%8.5f\n',data(i,j));  % %8.5八位浮点数，五位小数
%          else
%              fprintf(fid,'%8.5f\t',data(i,j));
%          end
%     end
%  end
% fclose(fid);
% 
% save data data


position0=[400,100,250];
position1=[400,200,250];
rpy0=[0 0 180];
rpy1=[0 0 180];

p0=[position0,rpy0];
p1=[position1,rpy1];

t=0:0.02:1;

p=t'*(p1-p0)+p0;

 fid = fopen('q_dis.txt','w');
 data_p=p;
 [m,n]=size(data_p);
 for i=1:1:m
     for j=1:1:n
         if j==n
             fprintf(fid,'%8.5f\n',data_p(i,j));  % %8.5八位浮点数，五位小数
         else
             fprintf(fid,'%8.5f\t',data_p(i,j));
         end
    end
 end
fclose(fid);
