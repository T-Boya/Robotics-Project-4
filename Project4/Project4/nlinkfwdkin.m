function robot=nlinkfwdkin(robot)

n = length(robot.joint_type);

joint_type = robot.joint_type;
q = robot.q;
H = robot.H;
P = robot.P;
T = eye(4, 4);
J = zeros(6, n);

for i = 1:n
    % calculate T
    h = H(1:3, i);
    R = expm(hat(h) * q(i)); p = P(1: 3, i); zeros(1, 3);
    J = phi(eye(3,3), T(1:3, 1:3) * p) * J;
    J(:,i) = [ T(1:3, 1:3) * h; zeros(3,1) ];
    T = T * [R p; zeros(1,3) 1];
    
end

robot.J = phi(eye(3,3), T(1:3, 1:3) * P(:, n+1)) * J;
robot.T = T*[eye(3,3) P(:, n+1); 0 0 0 1];

end