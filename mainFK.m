mode = 2; % 0 = 2D 2dof, 1 = 2D 5dof, 2 = sawyer

dist = 0.3;
a = [pi/2,0,pi/2, 0, pi/2];
lens = [1,1,1,1,1,1,1];
angles = zeros(7,1);
for k = 1:7
    for i = 0:0.1:2*pi   
        if mode == 1
            angles = [i, i*2, i*3];
            [P, joints, act] = FK3D(lens, angles);
            Ps = [joints; P'];
            hold off
            plot3(Ps(:,1), Ps(:,2), Ps(:,3), '.-', 'LineWidth', 2, 'MarkerSize',15);
            hold on
            plot3(act(1,:), act(2,:), act(3,:), '.-', 'LineWidth', 1.5, 'MarkerSize',10);
            grid on
            axis equal;
            axis([-3, 3, -3, 3, -3, 3]);
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            drawnow;

        elseif mode == 0

            angles = [i, i*2, i*3, i, i*2];
            lens = [1,1,1,1,1];
            [P, joints, act] = FK2D(lens, angles);
            Ps = [joints; P'];

            hold off
            plot3(Ps(:,1), Ps(:,2),Ps(:,3), '.-', 'LineWidth', 2, 'MarkerSize',15);
            hold on
            plot3(act(1,:), act(2,:), act(3,:), '.-', 'LineWidth', 1.5, 'MarkerSize',10);
            grid on
            axis equal;
            axis([-5, 5, -5, 5, -5, 5]);
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            drawnow;

        elseif mode == 2
            lens = [317, 192.5, 400, 168.5, 400, 136.3, 133.75];
%             angles = [i, i*2, i*3, i, i*2, i*3, i];
            angles(k) = i;
            angles = [0,0,0,0,0,0,0];
            [P, joints, act] = FKSawyer(lens, angles);
            Ps = [joints; P'];
            hold off
            plot3(Ps(:,1), Ps(:,2), Ps(:,3), '.-', 'LineWidth', 2, 'MarkerSize',15);
            hold on
            plot3(act(1,:), act(2,:), act(3,:), '.-', 'LineWidth', 1.5, 'MarkerSize',10);
            grid on
            axis equal;
            axis([-1300, 1300, -1300, 1300, -1300, 1300]);
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            drawnow;
            x = 1;
        end
    end   
end
