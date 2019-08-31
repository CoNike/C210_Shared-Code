% cobotta »úÆ÷ÈËÄæ½â
syms a2 a3 real;
syms d1 d3 d4 d5 d6 real;
syms q1 q2 q3 q4 q5 q6 real ;
syms nz nx ny ox oy oz  ax ay az px py pz real;

TG=[nx ox ax px;
    ny oy ay py;
    nz oz az pz;
    0 0 0 1];
T01=[cos(q1) 0 -sin(q1) 0;
    sin(q1) 0 cos(q1) 0;
    0 -1 0 d1;
    0 0 0 1];

T12=[-sin(q2) -cos(q2) 0 -a2*sin(q2);
    cos(q2) -sin(q2) 0 a2*cos(q2);
    0 0 1 0;
    0 0 0 1;];
T23=[cos(q3) 0 sin(q3) a3*cos(q3);
    sin(q3) 0 -cos(q3) a3*sin(q3);
    0 1 0 d3;
    0 0 0 1];

T34=[cos(q4) 0 -sin(q4) 0;
    sin(q4) 0 cos(q4) 0;
    0 -1 0 d4;
    0 0 0 1;];
T45=[cos(q5) 0 sin(q5) 0;
    sin(q5) 0 -cos(q5) 0;
    0 1 0 d5;
    0 0 0 1;];
T56=[cos(q6) -sin(q6) 0 0;
    sin(q6) cos(q6) 0 0;
    0 0 1 0;
    0 0 0 1;];
% 
T02=T01*T12;
T02=simplify(T02);

T03=T02*T23;
T03=simplify(T03);

T04=T03*T34;
T04=simplify(T04);

T04=T03*T34;
T04=simplify(T04);

T05=T04*T45;
T05=simplify(T05);

T06=T05*T56;
T06=simplify(T06);

% T56=DHstd_sy(0,0,d6,q6);
% T56=simplify(T56)
% vpa(T56,2)
% T45=DHstd_sy(pi/2,0,d5,q5);
% T45=simplify(T45)
% vpa(T45,2)
% T34=DHstd_sy(-pi/2,0,d4,q4);
% T34=simplify(T34)
% vpa(T34,2)
% T23=DHstd_sy(pi/2,a3,d3,q3);
% T23=simplify(T23)
% vpa(T23,2)
% T12=DHstd_sy(0,a2,0,pi/2+q2);
% T12=simplify(T12)
% vpa(T12,2)

% T01=DHstd_sy(-pi/2,0,d1,q1);
% vpa(T01,2)