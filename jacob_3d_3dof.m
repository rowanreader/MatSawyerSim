function jacob_3d_3dof

syms th1 th2 th3 l1 l2 l3 a1 a2 a3
 
T01 = DH(0, sym(pi)/2, l1, th1);
T12 = DH(0, sym(pi)/2, 0, th2 + pi/2);
T2P = DH(0, 0, l2+l3, th3);

T0P = T01*T12*T2P;
matrix = simplify(T0P);

% just take 1st 3 rows (x, y, z), last column
f = matrix(1:3, 4);
jacob = jacobian(f, [th1;th2;th3;]);
jacob = simplify(jacob)

th1 = pi/4; th2 = pi/3; th3 = pi/2; 
l1 = 1; l2 = 2; l3 = 1; 
eval(jacob)


