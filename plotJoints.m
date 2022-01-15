function plotJoints
L1 = 1;
len1 = 0;
a1 = 90;
d1 = L1;
th1_ = 0;


len2 = 0;
a2 = 90;
d2 = 0;
th2_ = 90;
L2 = 1;


L3 = 1;
len3 = 0;
a3 = 0;
d3 = L2+L3; 
th3_ = 0;

len4 = 0;
a4 = 0;
d4 = 0;

len5 = 0;
a5 = 0;
d5 = 0;

J1 = [0; 0; 0];

for i = 0:1%:360
    th1 = i;
    th2 = 2*i;
    th3 = floor(i/2);
    th4 = 3*i;
    th5 = i;
    
    th1 = 45;
    th2 = 45; 
    th3 = 45;
    
    % length of arm segment, alpha (angle btw z about x), d (change in x along
    % z), joint angle
    T1 = DH(len1, a1, d1, th1+ th1_);
    J2 = T1*[0,0,0,1]';
    J2 = J2(1:3);

    T2 = DH(len2, a2, d2, th2 + th2_);
    T12 = T1*T2;
    J3 = T12*[0,0,L2,1]';
    J3 = J3(1:3);
  

    T3 = DH(len3, a3, d3, th3 + th3_);
    T13 = T12*T3;
    P = T13*[0, 0, 0, 1]';
    P = P(1:3);
% 
%     T4 = DH(len4, a4, d4, th4);
%     T14 = T13*T4;
%     J4 = T14(1:3, 4);
% 
%     T5 = DH(len5, a5, d5, th5);
%     T15 = T14*T5;
%     J5 = T15(1:3, 4);

%     joints = [J0'; J1'; J2'; J3';];
%     figure();
%     plot3(joints(:,1), joints(:,2), joints(:,3), 'r');
%     axis equal;
%     J45 = [J4'; J5'];
%     J34 = [J3'; J4'];
    J3P = [J3'; P'];
    J23 = [J2'; J3'];
    J12 = [J1'; J2'];
    joints = [J12; J23; J3P];

    hold off
    plot3(joints(:,1), joints(:,2), joints(:,3), 'r.-', 'MarkerSize', 15)
    grid on;
    axis equal;
    % You can create any number of plots here after hold on
    axis([-3, 3,-3,3,-3, 3]);
    drawnow

end
disp(joints);
% end effector is J5
% where we want end effector to be
% goalX = 8;
% goalY = 7;
% 
% goalZ = 5;
% 
% xdiff = goalX - J5(1);
% ydiff = goalY - J5(2);
% 
% zdiff = goalZ - J5(3);
% 
% newAngle = atan2(ydiff/xdiff); % arc tan in 4 quads
