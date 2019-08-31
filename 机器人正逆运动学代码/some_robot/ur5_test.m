% É¾³ý
%ur5 test
d1=0.089195;d4=0.10915+0.003;d5=0.09465;d6=0.08230;
a2=-0.4250;a3=-0.39225;
L1= Link('revolute', 'd', d1,     'a', 0,        'alpha', pi/2,'offset',0);
L2= Link('revolute', 'd', 0,         'a', a2,    'alpha', 0,'offset',0);
L3= Link('revolute', 'd', 0,         'a', a3,   'alpha', 0,'offset',0);
L4= Link('revolute', 'd', d4,    'a', 0,        'alpha', pi/2,'offset',0);
L5= Link('revolute', 'd', d5,    'a', 0,        'alpha', -pi/2,'offset',0);
L6= Link('revolute', 'd', d6,     'a', 0,        'alpha', 0,'offset',0);
ur51=SerialLink([L1,L2,L3,L4,L5,L6]);
q1=[90.53 -89.98 94.11 270 -90 224.86]*pi/180;%[116.67 -478.69 397]
q2=[90.53 -89.98 86.03 270 -90 224.86]*pi/180;%[116.97 -490.52 465.62]
q3=[90.53 -88.03 86.03 273.86 -90 224.86]*pi/180;%[117 -497.44 442.31]
T1=ur51.fkine(q1);T1=T1.T;
T2=ur51.fkine(q2);T2=T2.T;
T3=ur51.fkine(q3);T3=T3.T;
delta1=norm(T1(1:3,4)*1000-[116.67 -478.69 397]');
delta2=norm(T2(1:3,4)*1000-[116.97 -490.52 465.62]');
delta3=norm(T3(1:3,4)*1000-[117 -497.44 442.31]');