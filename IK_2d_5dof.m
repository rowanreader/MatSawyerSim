function IK_2d_5dof

alpha = 0.2;   

% desired destination
pd = [(2.*rand(2,1) -1)*4];

% current position
angles = [pi/5, pi/4, pi/3, pi/2, pi/2];
lens = [1;1;1;1;1];
[p, joints, acts] = FK2D(lens, angles);

p = p(1:2);
q = 2*pi.*rand(5,1); % current config = angles in degrees
qOld = 0;
count = 0;
while norm(qOld-q) > 0.01 && count < 100
    count = count + 1;
    diff = pd-p;
    
    jacob = getJacob(q, lens);
    dq = pinv(jacob)*diff;
    qOld = q;
    
    q = q + alpha*dq;
    
    [p, joints, act] = FK2D(lens, q);
    p = p(1:2);
    joints = joints(:,1:2);
    Ps = [joints; p'];
    
    hold off
    plot(Ps(:,1), Ps(:,2), '.-', 'LineWidth', 2, 'MarkerSize',15);
    hold on
    plot(pd(1), pd(2), '*');
    plot(act(1,:), act(2,:), '.-', 'LineWidth', 1.5, 'MarkerSize',10);
    
    grid on
    axis equal;
    axis([-5, 5, -5, 5, -5, 5]);
    xlabel('X');
    ylabel('Y');
    
    drawnow;
    
end
end

function symbJacob = getJacob(q, lens)
    th1 = q(1); th2 = q(2); th3 = q(3); th4 = q(4); th5 = q(5);
    l1 = lens(1); l2 = lens(2); l3 = lens(3); l4 = lens(4); l5 = lens(5);
    
    symbJacob = [- l4*sin(th1 + th2 + th3 + th4) - l2*sin(th1 + th2) - l1*sin(th1) - l5*sin(th1 + th2 + th3 + th4 + th5) - l3*sin(th1 + th2 + th3), - l4*sin(th1 + th2 + th3 + th4) - l2*sin(th1 + th2) - l5*sin(th1 + th2 + th3 + th4 + th5) - l3*sin(th1 + th2 + th3), - l4*sin(th1 + th2 + th3 + th4) - l5*sin(th1 + th2 + th3 + th4 + th5) - l3*sin(th1 + th2 + th3), - l4*sin(th1 + th2 + th3 + th4) - l5*sin(th1 + th2 + th3 + th4 + th5), -l5*sin(th1 + th2 + th3 + th4 + th5)
l4*cos(th1 + th2 + th3 + th4) + l2*cos(th1 + th2) + l1*cos(th1) + l5*cos(th1 + th2 + th3 + th4 + th5) + l3*cos(th1 + th2 + th3),   l4*cos(th1 + th2 + th3 + th4) + l2*cos(th1 + th2) + l5*cos(th1 + th2 + th3 + th4 + th5) + l3*cos(th1 + th2 + th3),   l4*cos(th1 + th2 + th3 + th4) + l5*cos(th1 + th2 + th3 + th4 + th5) + l3*cos(th1 + th2 + th3),   l4*cos(th1 + th2 + th3 + th4) + l5*cos(th1 + th2 + th3 + th4 + th5),  l5*cos(th1 + th2 + th3 + th4 + th5)];
 
%     symbJacob = [- l4*sin(th1 + th2 + th3 + th4) - l2*sin(th1 + th2) - l1*sin(th1) - l5*sin(th1 + th2 + th3 + th4 + th5) - l3*sin(th1 + th2 + th3), - l4*sin(th1 + th2 + th3 + th4) - l2*sin(th1 + th2) - l5*sin(th1 + th2 + th3 + th4 + th5) - l3*sin(th1 + th2 + th3), - l4*sin(th1 + th2 + th3 + th4) - l5*sin(th1 + th2 + th3 + th4 + th5) - l3*sin(th1 + th2 + th3), - l4*sin(th1 + th2 + th3 + th4) - l5*sin(th1 + th2 + th3 + th4 + th5), -l5*sin(th1 + th2 + th3 + th4 + th5)
% l4*cos(th1 + th2 + th3 + th4) + l2*cos(th1 + th2) + l1*cos(th1) + l5*cos(th1 + th2 + th3 + th4 + th5) + l3*cos(th1 + th2 + th3),   l4*cos(th1 + th2 + th3 + th4) + l2*cos(th1 + th2) + l5*cos(th1 + th2 + th3 + th4 + th5) + l3*cos(th1 + th2 + th3),   l4*cos(th1 + th2 + th3 + th4) + l5*cos(th1 + th2 + th3 + th4 + th5) + l3*cos(th1 + th2 + th3),   l4*cos(th1 + th2 + th3 + th4) + l5*cos(th1 + th2 + th3 + th4 + th5),  l5*cos(th1 + th2 + th3 + th4 + th5)];

end
 
 