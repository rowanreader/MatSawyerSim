function IK_3d_3dof
alpha = 0.2;   

% desired destination
pd = [(2.*rand(3,1) -1)*3];

% current position
angles = [pi/5, pi/4, pi/3, pi/2, pi/2];
lens = [1;1;1;1;1];
[p, joints, acts] = FK3D(lens, angles);

p = p(1:3);
q = 2*pi.*rand(3,1); % current config = angles in rad
qOld = 0;
count = 0;
while norm(qOld-q) > 0.01 && count < 100
    count = count + 1;
    diff = pd-p;
    
    jacob = getJacob(q, lens);
    dq = pinv(jacob)*diff;
    qOld = q;
    
    q = q + alpha*dq;
    
    [p, joints, act] = FK3D(lens, q);
    p = p(1:3);
    joints = joints(:,1:3);
    Ps = [joints; p'];
    
    hold off
    plot3(Ps(:,1), Ps(:,2), Ps(:,3), '.-', 'LineWidth', 2, 'MarkerSize',15);
    hold on
    plot3(pd(1), pd(2), pd(3), '*');
    plot3(act(1,:), act(2,:), act(3,:), '.-', 'LineWidth', 1.5, 'MarkerSize',10);
    
    grid on
    axis equal;
    axis([-5, 5, -5, 5, -5, 5]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    drawnow;
    
end
end


function symbJacob = getJacob(q, lens)
    th1 = q(1); th2 = q(2); th3 = q(3);
    l1 = lens(1); l2 = lens(2); l3 = lens(3);
    
    symbJacob = [-cos(th2)*sin(th1)*(l2 + l3), -cos(th1)*sin(th2)*(l2 + l3), 0;
    cos(th1)*cos(th2)*(l2 + l3), -sin(th1)*sin(th2)*(l2 + l3), 0;
                           0,           cos(th2)*(l2 + l3), 0];
end
 
