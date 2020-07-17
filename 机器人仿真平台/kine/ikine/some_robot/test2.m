syms d1 d4 d5 d6 real;
syms  a2 a3 real;
syms q1 q2 q3 q4 q5 q6 real;
syms nx ny nz real;
syms ox oy oz real;
syms ax ay az real;
syms px py pz real;

T01=[cos(q1) 0 sin(q1) 0;
    sin(q1) 0 -cos(q1) 0;
    0 1 0 d1;
    0 0 0 1;];


T12=[ -sin(q2) cos(q2) 0 -a2*sin(q2);
    cos(q2) sin(q2) 0 a2*cos(q2);
    0 0 -1 0;
    0 0 0 1];

T23=[ cos(q3) sin(q3) 0 a3*cos(q3);
    sin(q3) -cos(q3) 0 a3*sin(q3);
    0 0 -1 0;
    0 0 0 1];

T34=[sin(q4) 0 cos(q4) 0;
    -cos(q4)  0 sin(q4) 0;
    0 -1.0  0 d4;
    0 0 0 1;];

T45=[cos(q5) 0 sin(q5) 0;
    sin(q5) 0 -cos(q5) 0;
    0 1 0 d5;
    0 0 0 1;];
    
T56=[cos(q6) -sin(q6) 0 0;
    sin(q6) cos(q6) 0 0;
    0 0 1 d6;
    0 0 0 1];

T06=T01*T12*T23*T34*T45*T56;
T06=simplify(T06);
TG=[nx ox ax px;
    ny oy ay py;
    nz oz az pz;
    0 0 0 1;];

T10=inv(T01);
T10=simplify(T10);

T65=inv(T56);
T65=simplify(T65);

T15=T12*T23*T34*T45;
T15=simplify(T15);

T150=T10*TG*T65;

T54=inv(T45);
T54=simplify(T54);

T14=T12*T23*T34;
T14=simplify(T14);

T140=T10*TG*T65*T54;
T140=simplify(T140);

m3=T14(1,4);T
n3=T14(2,4);
k3=m3^2+n3^2;
k3=simplify(k3);