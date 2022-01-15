dist = 0.3;
a = [90,90,0, 0, 0];
for i = 0:3:180   
%     angles = [i, i*2, i*3];
%     [P, joints, act] = FK3D(angles, a, dist);
%     Ps = [joints; P'];
%     hold off
%     plot3(Ps(:,1), Ps(:,2), Ps(:,3), '.-', 'LineWidth', 2, 'MarkerSize',15);
%     hold on
%     plot3(act(1,:), act(2,:), act(3,:), '.-', 'LineWidth', 1.5, 'MarkerSize',10);
%     grid on
%     axis equal;
%     axis([-3, 3, -3, 3, -3, 3]);
%     xlabel('X');
%     ylabel('Y');
%     zlabel('Z');
%     drawnow;

    angles = [i, i*2, i*3, i, i*2];
    [P, joints, act] = FK2D(angles, a);
    Ps = [joints; P'];
    
    hold off
    plot3(Ps(:,1), Ps(:,2), Ps(:,3), '.-', 'LineWidth', 2, 'MarkerSize',15);
    hold on
    plot3(act(1,:), act(2,:), act(3,:), '.-', 'LineWidth', 1.5, 'MarkerSize',10);
    grid on
    axis equal;
    axis([-5, 5, -5, 5, -5, 5]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    drawnow;
end

% given angles of the arm, lengths of the link, and offset (for 3D)
% find end point, joint locations, and claw location
function [P, joints, actuators] = FK3D(angles, a, dist)

    J0 = [0;0;0];
    l1 = 1;
    l2 = 1;
    l3 = 1;

    T1 = DH(0, a(1), l1, angles(1) + 90);
    J1 = T1*[0,0,0,1]';
    J1 = J1(1:3);


    T2 = DH(0, a(2), 0, angles(2) + 90);
    T12 = T1*T2;
    J2 = T12*[0,0,l2,1]';
    J2 = J2(1:3);


    T3 = DH(0, a(3), l2+l3, angles(3) + 0);
    T13 = T12*T3;
    P = T13*[0, 0, 0, 1]';
    P = P(1:3);


    J2P = [J2'; P'];
    J12 = [J1'; J2'];
    J01 = [J0'; J1'];
    joints = [J01; J12; J2P];

    A3 = [0; -dist; 0; 1];
    B3 = [0; -dist; dist; 1];
    C3 = [0; dist; 0; 1];
    D3 = [0; dist; dist; 1];

    A = T13*A3;
    B = T13*B3;
    C = T13*C3;
    D = T13*D3;

    actuators = [B(1:3) A(1:3) C(1:3) D(1:3)];

end


function [P, joints, actuators] = FK2D(angles, a)
    J0 = [0;0;0];
    dist = 0.3;

    T1 = DH(1, a(1), 0, angles(1));
    J1 = T1(1:3, 4);

    T2 = DH(1, a(2), 0, angles(2));
    T12 = T1*T2;
    J2 = T2(1:3, 4);
  

    T3 = DH(1, a(3), 0, angles(3));
    T13 = T12*T3;
    J3 = T3(1:3, 4);

    T4 = DH(1, a(4), 0, angles(4));
    T14 = T13*T4;
    J4 = T14(1:3, 4);

    T5 = DH(1, a(5), 0, angles(5));
    T15 = T14*T5;
    P = T15(1:3, 4);

    axis equal;
    J4P = [J4'; P'];    
    J34 = [J3'; J4'];
    J23 = [J2'; J3'];
    J12 = [J1'; J2'];
    J01 = [J0'; J1'];
    joints = [J01; J12; J23; J34; J4P];

    
    A3 = [0; -dist; 0; 1];
    B3 = [0; -dist; dist; 1];
    C3 = [0; dist; 0; 1];
    D3 = [0; dist; dist; 1];

    A = T15*A3;
    B = T15*B3;
    C = T15*C3;
    D = T15*D3;

    actuators = [B(1:3) A(1:3) C(1:3) D(1:3)];
end


