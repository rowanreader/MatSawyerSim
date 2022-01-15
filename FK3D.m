% given angles of the arm, lengths of the link, and offset (for 3D)
% find end point, joint locations, and claw location
function [P, joints, actuators] = FK3D(lens, angles)
    dist = 0.3;
    J0 = [0;0;0];
    l1 = lens(1);
    l2 = lens(2);
    l3 = lens(3);

    T1 = DH(0, pi/2, l1, angles(1));
    J1 = T1*[0,0,0,1]';
    J1 = J1(1:3);


    T2 = DH(0, pi/2, 0, angles(2) + pi/2);
    T12 = T1*T2;
    J2 = T12*[0,0,l2,1]';
    J2 = J2(1:3);


    T3 = DH(0, 0, l2+l3, angles(3));
    T13 = T12*T3;
    P = T13*[0, 0, 0, 1]';
    P = P(1:3);


%     J2P = [J2'; P'];
%     J12 = [J1'; J2'];
%     J01 = [J0'; J1'];
%     joints = [J01; J12; J2P];
joints = [J0'; J1'; J2'; P'];

    A3 = [0; dist; 0; 1];
    B3 = [0; dist; dist; 1];
    C3 = [0; -dist; 0; 1];
    D3 = [0; -dist; dist; 1];

    A = T13*A3;
    B = T13*B3;
    C = T13*C3;
    D = T13*D3;

    actuators = [B(1:3) A(1:3) C(1:3) D(1:3)];

end

