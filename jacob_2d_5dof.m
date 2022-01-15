function jacob_2d_5dof

syms th1 th2 th3 th4 th5 l1 l2 l3 l4 l5
 
T01 = DH(l1, 0, 0, th1);
T12 = DH(l2, 0, 0, th2);
T23 = DH(l3, 0, 0, th3);
T34 = DH(l4, 0, 0, th4);
T4P = DH(l5, 0, 0, th5);

T0P = T01*T12*T23*T34*T4P;
matrix = simplify(T0P);

% just take 1st 2 rows (x,y), last column
f = matrix(1:2, 4);
jacob = jacobian(f, [th1;th2;th3;th4;th5]);
jacob = simplify(jacob)

th1 = pi/4; th2 = pi/3; th3 = pi/2; th4 = pi/5; th5 = pi/4;  
l1 = 1; l2 = 2; l3 = 1; l4 = 1; l5 = 1;
eval(jacob)


