function n=nor_vec_p(p1,p2,p3)
%该函数用于求解不共线三点平面的法向量
% p1=p1(:)';
% p2=p2(:)';
% p3=p3(:)';
% vec=[p2-p1;
%     p3-p2];
% A=vec(1:2,1:2);
% b=vec(1:2,3);
% if abs(det(A))<10^-10
%     error('输入三点共线')
% end
% n_temp=-A\b;
% n=[n_temp;1]/norm([n_temp;1]);
p1=p1(:);
p2=p2(:);
p3=p3(:);
vec1=p2-p1;
vec2=p3-p2;
n=cross(vec1,vec2);
if norm(n)<10^-10
    error('三点共线，无法求解')
end
n=n/norm(n);
end