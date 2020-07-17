%安川机器人 逆运动学
% 
% <DHJoint name="Joint0" type="schilling" offset="0" d="0.33" a="0" alpha="0"/>
% <DHJoint name="Joint1" type="schilling" offset="0" d="0" a="0.088" alpha="-90"/>
% <DHJoint name="Joint2" type="schilling" offset="-90" d="0" a="0.31" alpha="180"/>
% <DHJoint name="Joint3" type="schilling" offset="0" d="0" a="0.040" alpha="-90"/>
% <DHJoint name="Joint4" type="schilling" offset="0" d="-0.305" a="0" alpha="90"/>
% <DHJoint name="Joint5" type="schilling" offset="0" d="0" a="0" alpha="-90"/>
% <Frame name="TCP" refframe="Joint5">
% <RPY>0 0 0</RPY>
% <Pos>0 0 -0.08</Pos>


% L1 = Link('d', 0.33, 'a', 0.088, 'alpha', -pi/2);      %offset',pi/2);
% L2 = Link('d', 0, 'a', 0.31, 'alpha', pi,'offset',-pi/2);     %offset',pi/4);
% L3 = Link('d', 0, 'a', 0.040, 'alpha', -pi/2);
% L4 = Link('d', -0.305, 'a', 0, 'alpha', pi/2);
% L5 = Link('d', 0, 'a', 0, 'alpha', -pi/2);
% L6 = Link('d', -0.08, 'a', 0, 'alpha', pi);
% L7 = Link('d', 0.037,'a',0,'alpha',0);
% robot=SerialLink([L1,L2,L3,L4,L5,L6,L7]);
% robot.name = 'Motoman robot';
% robot.display();
% theta=[0,0,0,0,-pi/2,0,0];
% robot.plot(theta);


syms d1 d4 d6 real;
syms a1 a2 a3 real;
syms q1 q2 q3 q4 q5 q6 real;
syms nx ny nz ox oy oz ax ay az px py pz real;
TG=[nx ox ax px;
    ny oy ay py;
    nz oz az pz;
    0 0 0 1;];
% d1=0.33;
% a1=0.088;
% a2=0.31;
% a3=0.04;
% d4=-0.305;
% d6=-0.08;
% q1=0;
% q2=0;
% q3=0;
% q4=0;
% q5=-pi/2;
% q6=0;

T01=[cos(q1) 0      -1.0*sin(q1)  a1*cos(q1);
    sin(q1)  0        cos(q1)      a1*sin(q1);
        0   -1           0                d1;
        0    0           0               1.0;];

T12=[      sin(q2), -1.0*cos(q2),    0,      a2*sin(q2);
      -1.0*cos(q2), -1.0*sin(q2),    0, -1.0*a2*cos(q2);
                 0,      0, -1.0,               0;
                 0,            0,    0,             1.0];
        
T23=[cos(q3),            0, -1.0*sin(q3), a3*cos(q3);
 sin(q3),                0,      cos(q3), a3*sin(q3);
       0,             -1.0,            0,          0;
       0,                0,            0,        1.0];
   
T34 =[ cos(q4),                0,      sin(q4),   0;
      sin(q4),                0, -1.0*cos(q4),   0;
            0,              1.0,            0,  d4;
            0,                0,            0, 1.0];
 T45 =[ cos(q5),                0 , -1.0*sin(q5),   0;
         sin(q5),                0,      cos(q5),   0;
               0,             -1.0,             0,   0;
               0,                0,            0, 1.0];
T56 = [ cos(q6),      sin(q6),       0,   0;
        sin(q6), -1.0*cos(q6),       0,   0;
              0,            0,    -1.0,  0;
              0,            0,       0, 1.0];

          
T02 = simplify(T01 * T12 );
T03 = simplify(T02 * T23);
T04 = simplify(T03 * T34);
T05 = simplify(T04 * T45);
T06 = simplify(T05 * T56);

T36=simplify(T34*T45*T56);
T360=simplify(inv(T03)*TG);
T030=simplify(TG*inv(T36));

T46=simplify(T45*T56);
T040=simplify(TG*inv(T46));

m1=simplify(T06(1,4)-a1*cos(q1));
n1=simplify(T06(2,4)-a1*sin(q1));
p1=simplify(T06(3,4)-d1);
 simplify(m1^2+n1^2+p1^2)
