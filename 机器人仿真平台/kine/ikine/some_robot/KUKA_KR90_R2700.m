%KUKA KR90 R2700 DH²ÎÊý

L1= Link('revolute', 'd', 0.675,     'a', 0.350,        'alpha', -pi/2,'offset',0);
L2= Link('revolute', 'd', 0,         'a', 1.15,    'alpha', 0,'offset',-pi/2);
L3= Link('revolute', 'd', 0,         'a', 0,   'alpha', -pi/2,'offset',0);
L4= Link('revolute', 'd', 1.2,    'a', 0,        'alpha', pi/2,'offset',0);
L5= Link('revolute', 'd', 0,    'a', 0,        'alpha', -pi/2,'offset',0);
L6= Link('revolute', 'd', 0.215,     'a', 0,        'alpha', 0,'offset',0);
KR90=SerialLink([L1,L2,L3,L4,L5,L6]);
q0=[0 0 0 0 0 0];
KR90.plot(q0)