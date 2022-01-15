function IKSawyer

alpha = 0.2;   
n = 13000;
% desired destination
pd = [(2.*rand(3,1) -1)*n/2];
% pd = [100;100;1000];

% current position
angles = [0,0,0,0,0,0,0];
lens = [3170; 1925; 4000; 1685; 4000; 1363; 1337.5];
[p, joints, acts] = FKSawyer(lens, angles);

p = p(1:3);
q = 2*pi.*rand(7,1); % current config = angles in degrees
q = [1,1,1,1,1,1,1];
qOld = [0,0,0,0,0,0,0];
count = 0;
while norm(qOld-q) > 0.05 && count < 70
    count = count + 1;
    diff = pd-p;
    
    jacob = getJacob(q, lens);
    dq = pinv(jacob)*diff;
    qOld = q;
    
    q = q + alpha*dq';
    
    [p, joints, act] = FKSawyer(lens, q);
    Ps = [joints; p'];
    
    hold off
    plot3(Ps(:,1), Ps(:,2), Ps(:,3), '.-', 'LineWidth', 2, 'MarkerSize',15);
    hold on
    plot3(pd(1), pd(2), pd(3), '*');
    plot3(act(1,:), act(2,:), act(3,:), '.-', 'LineWidth', 1.5, 'MarkerSize',10);
    
    grid on
    axis equal;
    axis([-n, n, -n, n, -n, n]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    drawnow;
    
end
end


function symbJacob = getJacob(q, lens)
    th1 = q(1); th2 = q(2); th3 = q(3); th4 = q(4); th5 = q(5); th6 = q(6); th7 = q(7);
    l1 = lens(1); l2 = lens(2); l3 = lens(3); l4 = lens(4); l5 = lens(5); l6 = lens(6); l7 = lens(7);
    symbJacob = ...
    [[l4*(cos(th1)*cos(th3) + sin(th1)*sin(th2)*sin(th3)) - l5*(cos(th2)*cos(th4)*sin(th1) - cos(th1)*sin(th3)*sin(th4) + cos(th3)*sin(th1)*sin(th2)*sin(th4)) - 81*sin(th1) + l6*(sin(th5)*(cos(th2)*sin(th1)*sin(th4) + cos(th1)*cos(th4)*sin(th3) - cos(th3)*cos(th4)*sin(th1)*sin(th2)) - cos(th5)*(cos(th1)*cos(th3) + sin(th1)*sin(th2)*sin(th3))) - l7*(cos(th6)*(cos(th2)*cos(th4)*sin(th1) - cos(th1)*sin(th3)*sin(th4) + cos(th3)*sin(th1)*sin(th2)*sin(th4)) + sin(th6)*(cos(th5)*(cos(th2)*sin(th1)*sin(th4) + cos(th1)*cos(th4)*sin(th3) - cos(th3)*cos(th4)*sin(th1)*sin(th2)) + sin(th5)*(cos(th1)*cos(th3) + sin(th1)*sin(th2)*sin(th3)))) - l3*cos(th2)*sin(th1), l6*(sin(th5)*(cos(th1)*sin(th2)*sin(th4) + cos(th1)*cos(th2)*cos(th3)*cos(th4)) + cos(th1)*cos(th2)*cos(th5)*sin(th3)) - l7*(cos(th6)*(cos(th1)*cos(th4)*sin(th2) - cos(th1)*cos(th2)*cos(th3)*sin(th4)) + sin(th6)*(cos(th5)*(cos(th1)*sin(th2)*sin(th4) + cos(th1)*cos(th2)*cos(th3)*cos(th4)) - cos(th1)*cos(th2)*sin(th3)*sin(th5))) - l5*(cos(th1)*cos(th4)*sin(th2) - cos(th1)*cos(th2)*cos(th3)*sin(th4)) - l3*cos(th1)*sin(th2) - l4*cos(th1)*cos(th2)*sin(th3), l6*(cos(th5)*(sin(th1)*sin(th3) + cos(th1)*cos(th3)*sin(th2)) + cos(th4)*sin(th5)*(cos(th3)*sin(th1) - cos(th1)*sin(th2)*sin(th3))) - l4*(sin(th1)*sin(th3) + cos(th1)*cos(th3)*sin(th2)) + l7*(sin(th6)*(sin(th5)*(sin(th1)*sin(th3) + cos(th1)*cos(th3)*sin(th2)) - cos(th4)*cos(th5)*(cos(th3)*sin(th1) - cos(th1)*sin(th2)*sin(th3))) + cos(th6)*sin(th4)*(cos(th3)*sin(th1) - cos(th1)*sin(th2)*sin(th3))) + l5*sin(th4)*(cos(th3)*sin(th1) - cos(th1)*sin(th2)*sin(th3)),   l7*(cos(th6)*(cos(th4)*sin(th1)*sin(th3) - cos(th1)*cos(th2)*sin(th4) + cos(th1)*cos(th3)*cos(th4)*sin(th2)) + cos(th5)*sin(th6)*(sin(th1)*sin(th3)*sin(th4) + cos(th1)*cos(th2)*cos(th4) + cos(th1)*cos(th3)*sin(th2)*sin(th4))) + l5*(cos(th4)*sin(th1)*sin(th3) - cos(th1)*cos(th2)*sin(th4) + cos(th1)*cos(th3)*cos(th4)*sin(th2)) - l6*sin(th5)*(sin(th1)*sin(th3)*sin(th4) + cos(th1)*cos(th2)*cos(th4) + cos(th1)*cos(th3)*sin(th2)*sin(th4)),   l6*(cos(th5)*(cos(th4)*sin(th1)*sin(th3) - cos(th1)*cos(th2)*sin(th4) + cos(th1)*cos(th3)*cos(th4)*sin(th2)) + sin(th5)*(cos(th3)*sin(th1) - cos(th1)*sin(th2)*sin(th3))) + l7*sin(th6)*(sin(th5)*(cos(th4)*sin(th1)*sin(th3) - cos(th1)*cos(th2)*sin(th4) + cos(th1)*cos(th3)*cos(th4)*sin(th2)) - cos(th5)*(cos(th3)*sin(th1) - cos(th1)*sin(th2)*sin(th3))), -l7*(sin(th6)*(sin(th1)*sin(th3)*sin(th4) + cos(th1)*cos(th2)*cos(th4) + cos(th1)*cos(th3)*sin(th2)*sin(th4)) + cos(th6)*(cos(th5)*(cos(th4)*sin(th1)*sin(th3) - cos(th1)*cos(th2)*sin(th4) + cos(th1)*cos(th3)*cos(th4)*sin(th2)) + sin(th5)*(cos(th3)*sin(th1) - cos(th1)*sin(th2)*sin(th3)))), 0],
    [81*cos(th1) + l5*(sin(th1)*sin(th3)*sin(th4) + cos(th1)*cos(th2)*cos(th4) + cos(th1)*cos(th3)*sin(th2)*sin(th4)) + l4*(cos(th3)*sin(th1) - cos(th1)*sin(th2)*sin(th3)) + l6*(sin(th5)*(cos(th4)*sin(th1)*sin(th3) - cos(th1)*cos(th2)*sin(th4) + cos(th1)*cos(th3)*cos(th4)*sin(th2)) - cos(th5)*(cos(th3)*sin(th1) - cos(th1)*sin(th2)*sin(th3))) + l7*(cos(th6)*(sin(th1)*sin(th3)*sin(th4) + cos(th1)*cos(th2)*cos(th4) + cos(th1)*cos(th3)*sin(th2)*sin(th4)) - sin(th6)*(cos(th5)*(cos(th4)*sin(th1)*sin(th3) - cos(th1)*cos(th2)*sin(th4) + cos(th1)*cos(th3)*cos(th4)*sin(th2)) + sin(th5)*(cos(th3)*sin(th1) - cos(th1)*sin(th2)*sin(th3)))) + l3*cos(th1)*cos(th2),                                             -sin(th1)*(l3*sin(th2) + l4*cos(th2)*sin(th3) + l5*cos(th4)*sin(th2) - l5*cos(th2)*cos(th3)*sin(th4) - l6*cos(th2)*cos(th5)*sin(th3) + l7*cos(th4)*cos(th6)*sin(th2) - l6*sin(th2)*sin(th4)*sin(th5) - l6*cos(th2)*cos(th3)*cos(th4)*sin(th5) - l7*cos(th2)*cos(th3)*cos(th6)*sin(th4) - l7*cos(th2)*sin(th3)*sin(th5)*sin(th6) + l7*cos(th5)*sin(th2)*sin(th4)*sin(th6) + l7*cos(th2)*cos(th3)*cos(th4)*cos(th5)*sin(th6)), l4*(cos(th1)*sin(th3) - cos(th3)*sin(th1)*sin(th2)) - l6*(cos(th5)*(cos(th1)*sin(th3) - cos(th3)*sin(th1)*sin(th2)) + cos(th4)*sin(th5)*(cos(th1)*cos(th3) + sin(th1)*sin(th2)*sin(th3))) - l7*(sin(th6)*(sin(th5)*(cos(th1)*sin(th3) - cos(th3)*sin(th1)*sin(th2)) - cos(th4)*cos(th5)*(cos(th1)*cos(th3) + sin(th1)*sin(th2)*sin(th3))) + cos(th6)*sin(th4)*(cos(th1)*cos(th3) + sin(th1)*sin(th2)*sin(th3))) - l5*sin(th4)*(cos(th1)*cos(th3) + sin(th1)*sin(th2)*sin(th3)), - l5*(cos(th2)*sin(th1)*sin(th4) + cos(th1)*cos(th4)*sin(th3) - cos(th3)*cos(th4)*sin(th1)*sin(th2)) - l7*(cos(th6)*(cos(th2)*sin(th1)*sin(th4) + cos(th1)*cos(th4)*sin(th3) - cos(th3)*cos(th4)*sin(th1)*sin(th2)) - cos(th5)*sin(th6)*(cos(th2)*cos(th4)*sin(th1) - cos(th1)*sin(th3)*sin(th4) + cos(th3)*sin(th1)*sin(th2)*sin(th4))) - l6*sin(th5)*(cos(th2)*cos(th4)*sin(th1) - cos(th1)*sin(th3)*sin(th4) + cos(th3)*sin(th1)*sin(th2)*sin(th4)), - l6*(cos(th5)*(cos(th2)*sin(th1)*sin(th4) + cos(th1)*cos(th4)*sin(th3) - cos(th3)*cos(th4)*sin(th1)*sin(th2)) + sin(th5)*(cos(th1)*cos(th3) + sin(th1)*sin(th2)*sin(th3))) - l7*sin(th6)*(sin(th5)*(cos(th2)*sin(th1)*sin(th4) + cos(th1)*cos(th4)*sin(th3) - cos(th3)*cos(th4)*sin(th1)*sin(th2)) - cos(th5)*(cos(th1)*cos(th3) + sin(th1)*sin(th2)*sin(th3))), -l7*(sin(th6)*(cos(th2)*cos(th4)*sin(th1) - cos(th1)*sin(th3)*sin(th4) + cos(th3)*sin(th1)*sin(th2)*sin(th4)) - cos(th6)*(cos(th5)*(cos(th2)*sin(th1)*sin(th4) + cos(th1)*cos(th4)*sin(th3) - cos(th3)*cos(th4)*sin(th1)*sin(th2)) + sin(th5)*(cos(th1)*cos(th3) + sin(th1)*sin(th2)*sin(th3)))), 0],
     [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     0,                                                         l4*sin(th2)*sin(th3) - l5*cos(th2)*cos(th4) - l3*cos(th2) - l7*cos(th2)*cos(th4)*cos(th6) - l5*cos(th3)*sin(th2)*sin(th4) - l6*cos(th5)*sin(th2)*sin(th3) + l6*cos(th2)*sin(th4)*sin(th5) - l6*cos(th3)*cos(th4)*sin(th2)*sin(th5) - l7*cos(th3)*cos(th6)*sin(th2)*sin(th4) - l7*cos(th2)*cos(th5)*sin(th4)*sin(th6) - l7*sin(th2)*sin(th3)*sin(th5)*sin(th6) + l7*cos(th3)*cos(th4)*cos(th5)*sin(th2)*sin(th6),                                                                                                                                                                                                                                                                 -cos(th2)*(l4*cos(th3) - l6*cos(th3)*cos(th5) + l5*sin(th3)*sin(th4) + l6*cos(th4)*sin(th3)*sin(th5) + l7*cos(th6)*sin(th3)*sin(th4) - l7*cos(th3)*sin(th5)*sin(th6) - l7*cos(th4)*cos(th5)*sin(th3)*sin(th6)),                                                                                                                                                      l5*sin(th2)*sin(th4) + l5*cos(th2)*cos(th3)*cos(th4) + l6*cos(th4)*sin(th2)*sin(th5) + l7*cos(th6)*sin(th2)*sin(th4) + l7*cos(th2)*cos(th3)*cos(th4)*cos(th6) - l6*cos(th2)*cos(th3)*sin(th4)*sin(th5) - l7*cos(th4)*cos(th5)*sin(th2)*sin(th6) + l7*cos(th2)*cos(th3)*cos(th5)*sin(th4)*sin(th6),                                                                                                                       l6*cos(th5)*sin(th2)*sin(th4) - l6*cos(th2)*sin(th3)*sin(th5) + l6*cos(th2)*cos(th3)*cos(th4)*cos(th5) + l7*cos(th2)*cos(th5)*sin(th3)*sin(th6) + l7*sin(th2)*sin(th4)*sin(th5)*sin(th6) + l7*cos(th2)*cos(th3)*cos(th4)*sin(th5)*sin(th6),                                                                                       l7*cos(th4)*sin(th2)*sin(th6) - l7*cos(th2)*cos(th3)*sin(th4)*sin(th6) + l7*cos(th2)*cos(th6)*sin(th3)*sin(th5) - l7*cos(th5)*cos(th6)*sin(th2)*sin(th4) - l7*cos(th2)*cos(th3)*cos(th4)*cos(th5)*cos(th6), 0]];
 
end                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    