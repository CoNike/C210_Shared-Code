%denvs087
% <DHJoint name="Joint1" type="schilling" offset="0" d="0.395" a="0.030" alpha="-90"/>
% <DHJoint name="Joint2" type="schilling" offset="-90" d="0" a="0.445" alpha="0"/>
% <DHJoint name="Joint3" type="schilling" offset="90" d="0" a="-0.020" alpha="90"/>
% <DHJoint name="Joint4" type="schilling" offset="0" d="0.430" a="0" alpha="-90"/>
% <DHJoint name="Joint5" type="schilling" offset="0" d="0" a="0" alpha="90"/>
% <RPY>0 0 0</RPY>
% <Pos>0 0 0.080</Pos>

% a1=0.03;a2=-0.445;a3=-0.02;
% d1=0.395;d4=0.430;d6=0.08+0.035;
% L1= Link('revolute', 'd', d1,     'a', a1,        'alpha', -pi/2,'offset',0);
% L2= Link('revolute', 'd', 0,         'a', a2,    'alpha', 0,'offset',pi/2);
% L3= Link('revolute', 'd', 0,         'a', a3,   'alpha', pi/2,'offset',-pi/2);
% L4= Link('revolute', 'd', d4,    'a', 0,        'alpha', -pi/2,'offset',0);
% L5= Link('revolute', 'd', 0,    'a', 0,        'alpha', pi/2,'offset',0);
% L6= Link('revolute', 'd', d6,     'a', 0,        'alpha', 0,'offset',0);
% den087=SerialLink([L1,L2,L3,L4,L5,L6]);
% q0=[-2.93 19.79 85.80 0.55 80 -3.13]*pi/180;   %[588.33 -29.04 602.99]
% T0=den087.fkine(q0);
% 
% qz=[0 0 0 0 0 0];
% Tz=den087.fkine(qz);
% 
% q1=q0+[0 0 +pi/2 0 0 0];
% T1=den087.fkine(q1);

%% 逆运动学求取
syms q1 q2 q3 q4 q5 q6 real;
syms a1 a2 a3 d1 d4 d6 real;
syms nx ny nz ox oy oz ax ay az px py pz real;
TG=[nx ox ax px;
    ny oy ay py;
    nz oz az pz;
    0 0 0 1;];
T01=[cos(q1)    0   -sin(q1) a1*cos(q1);
     sin(q1)    0   cos(q1)  a1*sin(q1);
      0         -1  0           d1;
      0         0   0           1;];
 T12=[-sin(q2)  -cos(q2)    0   -a2*sin(q2);
     cos(q2)    -sin(q2)    0   -a2*cos(q2);
     0          0           1       0;
     0          0           0       1;];
 T23=[sin(q3)   0   -cos(q3)    a3*sin(q3);
     -cos(q3)   0   -sin(q3)    -a3*cos(q3);
     0          1   0           0;
     0          0   0           1];
T34=[cos(q4)    0   -sin(q4) 0;
     sin(q4)    0   cos(q4)  0;
      0         -1  0        d4;
      0         0   0        1;];
T45=[cos(q5)    0   sin(q5) 0;
     sin(q5)    0   -cos(q5)  0;
      0         1  0        0;
      0         0   0        1;]; 
 T56=[cos(q6)   -sin(q6)  0   0;
     sin(q5)    cos(q6) 0     0;
      0         0  1        0;
      0         0   0        1;]; 
  T06=T01*T12*T23*T34*T45*T56;
  T06=simplify(T06)
  T16=T12*T23*T34*T45*T56;
  T16=simplify(T16)