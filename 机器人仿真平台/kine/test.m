%test
%函数测试文件


%% twistcross 测试 测试OK
%输入测试
% w1=[0 0 1]';r1=[0 0 0]';
% w2=[1 0 0]';r2=[1 0 1]';
% twist1=Twist('R',w1,r1);
% twist2=Twist('R',w2,r2);
% p=twistcross(twist1,twist2)
% twist1=[w1;r1];
% twist2=[w2;r2];
% p=twistcross(twist1,twist2)

%平行测试
% w1=[0 0 1]';r1=[0 0 0]';
% w2=[1 0 0]';r2=[1 0 1]';
% twist1=Twist('R',w1,r1);
% twist2=Twist('R',w1,r2);
% p=twistcross(twist1,twist2)

%不共面直线测试
% w1=[0 0 1]';r1=[0 0 0]';
% w2=[1 0 0]';r2=[1 1 1]';
% twist1=Twist('R',w1,r1);
% twist2=Twist('R',w2,r2);
% p=twistcross(twist1,twist2)