function T = DH(length, alpha, d, th)
T = [cos(th), -sin(th)*cos(alpha), sin(th)*sin(alpha),  length*cos(th);
    sin(th),  cos(th)*cos(alpha),  -cos(th)*sin(alpha), length*sin(th);
    0,              sin(alpha),                cos(alpha),                d;
    0,                  0,                          0,                    1];

