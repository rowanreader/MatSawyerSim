
function [P, joints, actuators] = FK2D(lens, angles)
    J0 = [0;0;0];
    dist = 0.3;
    num = 3;
    
    T1 = DH(lens(1), 0, 0, angles(1));
    J1 = T1(1:num, 4);

    T2 = DH(lens(2), 0, 0, angles(2));
    T12 = T1*T2;
    J2 = T12(1:num, 4);
  

    T3 = DH(lens(3), 0, 0, angles(3));
    T13 = T12*T3;
    J3 = T13(1:num, 4);

    T4 = DH(lens(4), 0, 0, angles(4));
    T14 = T13*T4;
    J4 = T14(1:num, 4);

    T5 = DH(lens(5), 0, 0, angles(5));
    T1P = T14*T5;
    P = T1P(1:num, 4);

    axis equal;
    J4P = [J4'; P'];    
    J34 = [J3'; J4'];
    J23 = [J2'; J3'];
    J12 = [J1'; J2'];
    J01 = [J0'; J1'];
    joints = [J01; J12; J23; J34; J4P];

    
    A3 = [0;-dist; 0; 1];
    B3 = [0;-dist; dist; 1];
    C3 = [0;dist; 0; 1];
    D3 = [0;dist; dist; 1];

%     T15 = T15(1:3, 1:3);
    A = T1P*A3;
    B = T1P*B3;
    C = T1P*C3;
    D = T1P*D3;

    actuators = [B(1:3) A(1:3) C(1:3) D(1:3)];
    
    
end

