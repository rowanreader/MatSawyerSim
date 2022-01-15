function jacobSawyer

syms th1 th2 th3 th4 th5 th6 th7 l1 l2 l3 l4 l5 l6 l7 a1 a2 a3 a4 a5 a6 a7
 
T01 = DH(81, -sym(pi)/2, l1, th1);
T12 = DH(0, -sym(pi)/2, 0, th2 + 3*sym(pi)/2);
T23 = DH(0, -sym(pi)/2, l3, th3);
T34 = DH(0, -sym(pi)/2, l4, th4 + sym(pi));
T45 = DH(0, -sym(pi)/2, l5, th5);
T56 = DH(0, -sym(pi)/2, l6, th6 + sym(pi));
T67 = DH(0, 0, l7, th7 + 3*sym(pi)/2);

T0P = T01*T12*T23*T34*T45*T56*T67;
matrix = simplify(T0P);

% just take 1st 3 rows (x, y, z), last column
f = matrix(1:3, 4);
jacob = jacobian(f, [th1;th2;th3;th4;th5;th6;th7]);
jacob = simplify(jacob)
size(jacob)

% th1 = pi/4; th2 = pi/3; th3 = pi/2; 
% l1 = 1; l2 = 2; l3 = 1; 
% eval(jacob)


