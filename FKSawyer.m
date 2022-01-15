% given angles of the arm, lengths of the link, and offset (for 3D)
% find end point, joint locations, and claw location
function [P, joints, actuators] = FKSawyer(lens, angles)
    dist = 700;
    J0 = [0;0;0];
    l1 = lens(1);
    l2 = lens(2);
    l3 = lens(3);
    l4 = lens(4);
    l5 = lens(5);
    l6 = lens(6);
    l7 = lens(7);

    T1 = DH(81, - pi/2, l1, angles(1));
    J1 = T1*[0,0,0,1]';
    J1 = J1(1:3);

    J1_ = T1*[-81, 0, 0, 1]';
    J1_ = J1_(1:3);

    T2 = DH(0, - pi/2, l2, 3*pi/2 + angles(2));
    T12 = T1*T2;
    J2 = T12*[0,0,0,1]';
    J2 = J2(1:3);
    
    
    T3 = DH(0, -pi/2, l3, angles(3));
    T13 = T12*T3;
    J3 = T13*[0, 0, 0, 1]';
    J3 = J3(1:3);

    
    T4 = DH(0, -pi/2, l4, angles(4) + pi);
    T14 = T13*T4;
    J4 = T14*[0, 0, 0, 1]';
    J4 = J4(1:3);
    
    T5 = DH(0, -pi/2, l5, angles(5));
    T15 = T14*T5;
    J5 = T15*[0, 0, 0, 1]';
    J5 = J5(1:3);
    
    T6 = DH(0, -pi/2, l6, angles(6) + pi);
    T16 = T15*T6;
    J6 = T16*[0, 0, 0, 1]';
    J6 = J6(1:3);
    
    T7 = DH(0, 0, l7, angles(7) + 3*pi/2);
    T1P = T16*T7;
    J7 = T1P*[0, 0, 0, 1]';
    P = J7(1:3);
    
    joints = [J0'; J1_'; J1'; J2'; J3'; J4'; J5'; J6';];

    A3 = [dist; 0; 0; 1];
    B3 = [dist; 0; dist; 1];
    C3 = [-dist; 0; 0; 1];
    D3 = [-dist; 0; dist; 1];

    A = T1P*A3;
    B = T1P*B3;
    C = T1P*C3;
    D = T1P*D3;

    actuators = [B(1:3) A(1:3) C(1:3) D(1:3)];

end

