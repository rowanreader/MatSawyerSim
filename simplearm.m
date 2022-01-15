clear;clc;close all;
% Robot params
l = 1;

% Robot initialization
th = 0;


figure


for t = 0:0.01:5
    th = t;
    % FK (q will be my final configuration [x;y] or [x;y;z] if 3D
    q = [cos(th); sin(th)];
    r = [cos(2*th); sin(2*th)];
    
    % Plotting stuff
    o = [0;0]; % origin
    
    P1 = [o q]; %Points to plot, each column is a point. Right now I have the origin and the "actuator" in my 1 dof robot
    P2 = [q r];
    
    hold off
    plot(P1(1,:), P1(2,:), '.', 'MarkerSize',15);
    hold on
    plot(P2(1,:), P2(2,:), '.', 'MarkerSize',15);
    plot(P1(1,:), P1(2,:),'-');
    plot(P2(1,:), P2(2,:), '-');
    % You can create any number of plots here after hold on
    axis([-2, 2,-2, 2]);
    drawnow
end


