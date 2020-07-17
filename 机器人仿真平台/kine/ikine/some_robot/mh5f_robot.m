t01=DHstd(0,0,0.33,0);
t12=DHstd(-90,0.088,0,0);
t23=DHstd(180,0.31,0,-90);
t34=DHstd(-90,0.040,0,0);
t45=DHstd(90,0,-0.305,0);
t56=DHstd(-90,0,0,90);
t67=DHstd(0,0,0.024,0);
t=t01*t12*t23*t34*t45*t56

L1= Link('revolute', 'd', 0.33,     'a', 0.088,        'alpha',- pi/2,'offset',0);
L2= Link('revolute', 'd', 0,         'a', 0.31,    'alpha', pi,'offset',-pi/2);
L3= Link('revolute', 'd', 0,         'a', 0.40,   'alpha', -pi/2,'offset',0);
L4= Link('revolute', 'd',-0.305,    'a', 0,        'alpha', pi/2,'offset',0);
L5= Link('revolute', 'd', 0,    'a', 0,        'alpha', -pi/2,'offset',0);
L6= Link('revolute', 'd', 0.024,     'a', 0,        'alpha', 0,'offset',0);

mh5f=SerialLink([L1,L2,L3,L4,L5,L6]);